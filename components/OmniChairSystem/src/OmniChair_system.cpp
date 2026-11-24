#include "OmniChair_system.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>

using namespace std::chrono_literals;

OmniChairSystem::OmniChairSystem()
    : BaseComponent("OmniChair", espp::Logger::Verbosity::INFO) {}

bool OmniChairSystem::start() {
  if (!init_mcps())
    return false;
  if (!init_ethernet())
    return false;

  logger_.info("Waiting for the Ethernet link (is the joystick plugged in?)");
  while (!espp::Esp32P4Eth::get().is_ethernet_connected())
    std::this_thread::sleep_for(100ms);

  if (!init_rtps() || !init_pubsub())
    return false;

  control_task_ = espp::Task::make_unique({
      .callback = [this](std::mutex &m, std::condition_variable &cv) -> bool {
        return control_loop(m, cv);
      },
      .task_config = {.name = "OmniChair Control", .stack_size_bytes = 6144, .priority = 10},
  });
  control_task_->start();
  return true;
}

std::unique_ptr<espp::Basicmicro> OmniChairSystem::make_mcp(uart_port_t port, int tx_pin,
                                                            int rx_pin) {
  uart_config_t uart_config = {};
  uart_config.baud_rate = kUartBaud;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(port, 256, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  return std::make_unique<espp::Basicmicro>(espp::Basicmicro::Config{
      .address = 0x80, // default packet serial address (0x80 - 0x87)
      .write =
          [port](std::span<const uint8_t> data) {
            uart_flush_input(port); // drop any late / leftover reply bytes
            const int written =
                uart_write_bytes(port, reinterpret_cast<const char *>(data.data()), data.size());
            return written == static_cast<int>(data.size());
          },
      .read = [port](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
        const int read =
            uart_read_bytes(port, data.data(), data.size(), pdMS_TO_TICKS(timeout.count()));
        return read < 0 ? 0 : static_cast<size_t>(read);
      },
      // must be >= 10 ms: a 10 ms quiet gap is also what clears the
      // controller's packet buffer after a communication error
      .timeout = 100ms,
      .log_level = espp::Logger::Verbosity::INFO,
  });
}

bool OmniChairSystem::init_mcps() {
  mcp1_ = make_mcp(kMcp1Port, kMcp1TxPin, kMcp1RxPin);
  mcp2_ = make_mcp(kMcp2Port, kMcp2TxPin, kMcp2RxPin);

  std::error_code ec;
  std::string version;
  if (!mcp1_->read_firmware_version(version, ec)) {
    logger_.error("MCP1: could not read firmware version: {}", ec.message());
    return false;
  }
  logger_.info("MCP1 firmware: '{}'", version);
  if (!mcp2_->read_firmware_version(version, ec)) {
    logger_.error("MCP2: could not read firmware version: {}", ec.message());
    return false;
  }
  logger_.info("MCP2 firmware: '{}'", version);
  return true;
}

void OmniChairSystem::set_targets(float target1, float target2, float target3) {
  target1_ = std::clamp(target1, -1.0f, 1.0f);
  target2_ = std::clamp(target2, -1.0f, 1.0f);
  target3_ = std::clamp(target3, -1.0f, 1.0f);
}

int16_t OmniChairSystem::to_duty(float target) const {
  return static_cast<int16_t>(target * kMaxDuty * 32767.0f);
}

void OmniChairSystem::update_motors() {
  using Axis = espp::Basicmicro::Axis;
  std::error_code ec;
  // wheel 1 = MCP1 M1, wheel 2 = MCP1 M2 (one packet for both), wheel 3 = MCP2 M1
  // wheel 1 = front
  // wheel 2 = rear left
  // wheel 3 = rear right
  if (!mcp1_->drive_both_duty(to_duty(-target1_), to_duty(target2_), ec))
    logger_.error("MCP1 drive failed: {}", ec.message());
  if (!mcp2_->drive_duty(Axis::M1, to_duty(-target3_), ec))
    logger_.error("MCP2 drive failed: {}", ec.message());
}

void OmniChairSystem::stop_motors() {
  set_targets(0.0f, 0.0f, 0.0f);
  vxfilter = vyfilter = omegafilter = 0.0;
  pre_vxfilter = pre_vyfilter = pre_omegafilter = 0.0;
  wheel1 = wheel2 = wheel3 = 0.0f;
  prevWheel1 = prevWheel2 = prevWheel3 = 0.0f;
}

float OmniChairSystem::filter(float rawValue, float w, float prevValue) {
  return w * rawValue + (1.0 - w) * prevValue;
}

float OmniChairSystem::selectFilter(float target, float previous) {
  return (std::fabs(target) < std::fabs(previous)) ? Kfvel_decel : Kfvel_accel;
}

float OmniChairSystem::chooseComp(float pos, float neg, float value) {
  return (value >= 0.0f) ? pos : neg;
}

float OmniChairSystem::limitAcceleration(float previous, float target, float accelLimit,
                                         float decelLimit) {
  float delta = target - previous;
  bool decelerating = (std::fabs(target) < std::fabs(previous));
  bool reversing = (previous * target < -0.001);
  float limit;
  if (reversing)
    limit = max_reversal;
  else if (decelerating)
    limit = decelLimit;
  else
    limit = accelLimit;
  if (delta > limit)
    delta = limit;
  if (delta < -limit)
    delta = -limit;
  return previous + delta;
}

float OmniChairSystem::applyDeadband(float w, float minDuty) {
  if (minDuty < 0.001)
    return w;
  if (w > 0.001)
    return minDuty + w * (1.0 - minDuty);
  if (w < -0.001)
    return -minDuty + w * (1.0 - minDuty);
  return 0.0;
}

bool OmniChairSystem::init_ethernet() {
  espp::Esp32P4Eth::EthernetConfig config{};
  config.mode = espp::Esp32P4Eth::DhcpMode::SERVER; // the joystick is a DHCP client
  config.server_config.ip_info.ip.addr = ESP_IP4TOADDR(192, 168, 4, 1);
  config.server_config.ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
  config.server_config.ip_info.gw.addr = ESP_IP4TOADDR(0, 0, 0, 0); // no router on this link
  if (!espp::Esp32P4Eth::get().initialize_ethernet(config)) {
    logger_.error("Failed to initialize Ethernet");
    return false;
  }
  return true;
}

bool OmniChairSystem::init_rtps() {
  rtps_participant_ = std::make_unique<espp::RtpsParticipant>(espp::RtpsParticipant::Config{
      .interface_address = kIpAddress,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!rtps_participant_->start()) {
    logger_.error("Failed to start the RTPS participant");
    rtps_participant_.reset();
    return false;
  }
  return true;
}

bool OmniChairSystem::init_pubsub() {
  using Reliability = espp::RtpsParticipant::Reliability;
  joystick_subscriber_ = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
      *rtps_participant_,
      espp::Subscriber<rammp::XYTwist>::Config{
          .topic = rammp::kJoystickXYTwist.name,
          .type_name = rammp::kJoystickXYTwist.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::XYTwist &sample) { handle_joystick_message(sample); },
      });
  if (!joystick_subscriber_->is_valid()) {
    logger_.error("Failed to create the joystick subscriber");
    return false;
  }

  status_pub_ = std::make_unique<espp::Publisher<rammp::MibStatus>>(
      *rtps_participant_,
      espp::Publisher<rammp::MibStatus>::Config{.topic = rammp::kMibStatus.name,
                                                .type_name = rammp::kMibStatus.type});
  if (!status_pub_->is_valid()) {
    logger_.error("Failed to create the status publisher");
    return false;
  }

  drive_sub_ = std::make_unique<espp::Subscriber<rammp::DriveCommand>>(
      *rtps_participant_,
      espp::Subscriber<rammp::DriveCommand>::Config{
          .topic = rammp::kDriveCommand.name,
          .type_name = rammp::kDriveCommand.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::DriveCommand &c) {
            if (c.request == rammp::DriveRequest::ENABLE) {
              profile_ = rammp::DriveProfile::NORMAL; // for now, ignores all other drive modes and
                                                      // defaults to normal
              state_ = State::ENABLED;
            } else if (c.request == rammp::DriveRequest::DISABLE) {
              state_ = State::IDLE;
            }
          }});
  if (!drive_sub_->is_valid()) {
    logger_.error("Failed to create the drive command subscriber");
    return false;
  }

  return true;
}

// RTPS receive task: just record the sample, never block here.
void OmniChairSystem::handle_joystick_message(const rammp::XYTwist &sample) {
  std::lock_guard<std::mutex> lock(joystick_mutex_);
  latest_joystick_ = sample;
  last_joystick_time_ = std::chrono::steady_clock::now();
}

// Control task, ~50 Hz: drive on the latest sample, or stop if the joystick went quiet.
bool OmniChairSystem::control_loop(std::mutex &m, std::condition_variable &cv) {
  rammp::XYTwist stick{};
  std::chrono::steady_clock::time_point stamp{};
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    stick = latest_joystick_;
    stamp = last_joystick_time_;
  }

  const auto since_stick = std::chrono::steady_clock::now() - stamp;
  if (since_stick > kJoystickLostTimeout) // e.g. 1s: joystick gone, require a new unlock
    state_ = State::IDLE;

  if (state_ != State::ENABLED || since_stick > kJoystickTimeout)
    stop_motors();
  else
    apply_stick(stick);

  update_motors();

  // Status: at once when the state changes (the HMI waits only ~750 ms for an
  // answer), otherwise every 500 ms by the clock, not by loop count.
  static State last_sent = State::IDLE;
  static auto last_status = std::chrono::steady_clock::time_point{};
  const State now_state = state_;
  const auto now = std::chrono::steady_clock::now();
  if (now_state != last_sent || now - last_status >= 500ms) {
    last_sent = now_state;
    last_status = now;
    rammp::MibStatus s{};
    s.system_state =
        now_state == State::ENABLED ? rammp::MibSystemState::ENABLED : rammp::MibSystemState::IDLE;
    s.active_profile = profile_;
    s.seq = ++status_seq_;
    if (!status_pub_->publish(s))
      logger_.warn("Failed to publish MibStatus");
  }

  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, 20ms);
  return false;
}

void OmniChairSystem::apply_stick(const rammp::XYTwist &sample) {
  // logger_.info("Joystick x={:.2f} y={:.2f} twist={:.2f} buttons={}", sample.x, sample.y,
  //              sample.twist, static_cast<uint32_t>(sample.buttons));

  const float joyx = sample.x;
  const float joyy = sample.y;
  const float joyz = sample.twist;

  // map joystick axes to chair velocities based on drive mode.
  // currently bc GUI side not impl, by default in omni mode
  float cmd_vx, cmd_vy, cmd_omega;
  float mode_accel_omega, mode_decel_omega;

  int mode = 1;
  if (mode == 0) {
    // wheelchair: Y=drive, X=turn, Z=strafe
    cmd_vy = joyy * max_vy;
    cmd_omega = joyx * max_turn;
    cmd_vx = -joyz * max_vx_wc;
    mode_accel_omega = max_accel * OMEGA_ACCEL_BOOST;
    mode_decel_omega = max_decel * OMEGA_ACCEL_BOOST;
  } else {
    cmd_vx = joyx * max_vx_omni;
    cmd_vy = joyy * max_vy;
    cmd_omega = joyz * max_omega;

    mode_accel_omega = max_accel * OMEGA_ACCEL_BOOST * 0.8;
    mode_decel_omega = max_decel * OMEGA_ACCEL_BOOST * 0.8;
  }

  // velocity filtering
  double kvx = selectFilter(cmd_vx, pre_vxfilter);
  double kvy = selectFilter(cmd_vy, pre_vyfilter);
  double kw = selectFilter(cmd_omega, pre_omegafilter);
  vxfilter = filter(cmd_vx, kvx, pre_vxfilter);
  vyfilter = filter(cmd_vy, kvy, pre_vyfilter);
  omegafilter = filter(cmd_omega, kw, pre_omegafilter);

  bool joy_neutral =
      (std::fabs(joyx) < 0.05) && (std::fabs(joyy) < 0.05) &&
      (std::fabs(joyz) < 0.05); // need to cross check these dead bands against joystick dead bands

  float vx_decel = max_decel;
  float vy_decel = max_decel;
  float w_decel = mode_decel_omega;
  if (joy_neutral) {
    vx_decel = hard_decel;
    vy_decel = hard_decel;
    w_decel = hard_decel * (mode_decel_omega / max_decel);
  }

  // start accel boost
  float cur_speed = std::sqrt(pre_vxfilter * pre_vxfilter + pre_vyfilter * pre_vyfilter);
  float vx_accel = max_accel;
  float vy_accel = max_accel;
  if (cur_speed < START_ACCEL_RAMP_VEL) {
    float blend_a = cur_speed / START_ACCEL_RAMP_VEL;
    vx_accel = start_accel + blend_a * (max_accel - start_accel);
    vy_accel = start_accel + blend_a * (max_accel - start_accel);
  }

  vxfilter = limitAcceleration(pre_vxfilter, vxfilter, vx_accel, vx_decel);
  vyfilter = limitAcceleration(pre_vyfilter, vyfilter, vy_accel, vy_decel);
  omegafilter = limitAcceleration(pre_omegafilter, omegafilter, mode_accel_omega, w_decel);

  // combined accel/decel cap for linear motion
  float dvx = vxfilter - pre_vxfilter;
  float dvy = vyfilter - pre_vyfilter;
  float accel_mag = std::sqrt(dvx * dvx + dvy * dvy);

  float linear_delta_cap = joy_neutral ? hard_decel : max_decel;

  if (accel_mag > linear_delta_cap) {
    float accel_scale = linear_delta_cap / accel_mag;
    vxfilter = pre_vxfilter + dvx * accel_scale;
    vyfilter = pre_vyfilter + dvy * accel_scale;
  }

  // total xy velocity magnitude cap
  float v_mag = std::sqrt(vxfilter * vxfilter + vyfilter * vyfilter);
  if (v_mag > max_v_mag) {
    float v_scale = max_v_mag / v_mag;
    vxfilter *= v_scale;
    vyfilter *= v_scale;
  }

  pre_vxfilter = vxfilter;
  pre_vyfilter = vyfilter;
  pre_omegafilter = omegafilter;

  // Inverse kinematics: (x, y, twist) -> each wheel's speed along its drive direction.
  raw_w1 = -vxfilter * std::cos(alpha1) + vyfilter * std::sin(alpha1) - R * omegafilter;
  raw_w2 = -vxfilter * std::cos(alpha2) + vyfilter * std::sin(alpha2) - R * omegafilter;
  raw_w3 = -vxfilter * std::cos(alpha3) + vyfilter * std::sin(alpha3) - R * omegafilter;

  // strafe blend factor
  float totalV = std::fabs(vxfilter) + std::fabs(vyfilter);
  float blend = 0.0;
  if (totalV > 0.01) {
    blend = std::fabs(vxfilter) / totalV;
    blend = std::pow(blend, 1.8);
  }

  // per-wheel direction-specific compensation
  float comp_w1_strafe;
  if (vxfilter < 0)
    comp_w1_strafe = COMP_W1_STRAFE_RIGHT;
  else
    comp_w1_strafe = COMP_W1_STRAFE_LEFT;
  float comp_w1 = COMP_W1_FWD + blend * (comp_w1_strafe - COMP_W1_FWD);

  float comp_w2_strafe;
  float comp_w3_strafe;
  if (vxfilter < 0) {
    comp_w2_strafe = COMP_W2_STRAFE_RIGHT;
    comp_w3_strafe = COMP_W3_STRAFE_RIGHT;
  } else {
    comp_w2_strafe = COMP_W2_STRAFE_LEFT;
    comp_w3_strafe = COMP_W3_STRAFE_LEFT;
  }

  float base_w2 = chooseComp(COMP_W2_POS, COMP_W2_NEG, raw_w2);
  float comp_w2 = base_w2 + blend * (comp_w2_strafe - base_w2);

  float base_w3 = chooseComp(COMP_W3_POS, COMP_W3_NEG, raw_w3);
  float comp_w3 = base_w3 + blend * (comp_w3_strafe - base_w3);

  wheel1 = raw_w1 * comp_w1;
  wheel2 = raw_w2 * comp_w2;
  wheel3 = raw_w3 * comp_w3;

  // wheel-level accel limiting
  float w1_accel = max_accel * (1.0 + blend * 0.8);
  float w1_decel = max_decel * (1.0 + blend * 3.0);
  wheel1 = limitAcceleration(prevWheel1, wheel1, w1_accel, w1_decel);
  wheel2 = limitAcceleration(prevWheel2, wheel2, max_accel, max_decel);
  wheel3 = limitAcceleration(prevWheel3, wheel3, max_accel, max_decel);
  prevWheel1 = wheel1;
  prevWheel2 = wheel2;
  prevWheel3 = wheel3;

  // normalize so no wheel clips
  float maxWheel = std::max(std::max(std::fabs(wheel1), std::fabs(wheel2)), std::fabs(wheel3));
  if (maxWheel > 1.0) {
    wheel1 /= maxWheel;
    wheel2 /= maxWheel;
    wheel3 /= maxWheel;
  }

  // per-wheel deadband
  wheel1 = applyDeadband(wheel1, W1_MIN_DUTY);
  wheel2 = applyDeadband(wheel2, W2_MIN_DUTY);
  wheel3 = applyDeadband(wheel3, W3_MIN_DUTY);

  set_targets(wheel1, wheel2, wheel3);
}