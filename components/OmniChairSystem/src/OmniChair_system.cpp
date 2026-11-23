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
  // using Axis = espp::Basicmicro::Axis;
  std::error_code ec;
  // wheel 1 = MCP1 M1, wheel 2 = MCP1 M2 (one packet for both), wheel 3 = MCP2 M1
  // wheel 1 = front
  // wheel 2 = rear left
  // wheel 3 = rear right
  // drving commands
  if (!mcp1_->drive_both_duty(to_duty(-target1_), to_duty(target2_), ec))
    logger_.error("MCP1 drive failed: {}", ec.message());
  if (!mcp2_->drive_both_duty(to_duty(-target3_), to_duty(target4_), ec))
    logger_.error("MCP2 drive failed: {}", ec.message());
  // TODO MCP3: send to_duty(target5_) for the footrest
}

void OmniChairSystem::stop_motors() { set_targets(0.0f, 0.0f, 0.0f); }

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

  seat_subscriber_ = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
      *rtps_participant_,
      espp::Subscriber<rammp::XYTwist>::Config{.topic = "rammp/joystick/seat/xy_twist",
                                               .type_name = rammp::kJoystickXYTwist.type,
                                               .reliability = Reliability::BEST_EFFORT,
                                               .on_message = [this](const rammp::XYTwist &s) {
                                                 std::lock_guard<std::mutex> lock(joystick_mutex_);
                                                 latest_seat_ = s;
                                                 last_seat_time_ = std::chrono::steady_clock::now();
                                               }});
  if (!seat_subscriber_->is_valid()) {
    logger_.error("Failed to create the seat subscriber");
    return false;
  }

  swivel_subscriber_ = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
      *rtps_participant_,
      espp::Subscriber<rammp::XYTwist>::Config{.topic = "rammp/joystick/swivel/xy_twist",
                                               .type_name = rammp::kJoystickXYTwist.type,
                                               .reliability = Reliability::BEST_EFFORT,
                                               .on_message = [this](const rammp::XYTwist &s) {
                                                 std::lock_guard<std::mutex> lock(joystick_mutex_);
                                                 latest_footrest_ = s;
                                                 last_footrest_time_ =
                                                     std::chrono::steady_clock::now();
                                               }});
  if (!swivel_subscriber_->is_valid()) {
    logger_.error("Failed to create the swivel subscriber");
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
              profile_ = rammp::DriveProfile::NORMAL; // for now, ignores all other drive modes
                                                      // and defaults to normal
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
  rammp::XYTwist stick{}, seat{}, foot{};
  std::chrono::steady_clock::time_point stamp{}, seat_stamp{}, foot_stamp{};
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    stick = latest_joystick_;
    stamp = last_joystick_time_;
    seat = latest_seat_;
    seat_stamp = last_seat_time_;
    foot = latest_footrest_;
    foot_stamp = last_footrest_time_;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto since_stick = now - stamp;
  if (since_stick > kJoystickLostTimeout)
    state_ = State::IDLE;
  const bool enabled = state_ == State::ENABLED;

  // Wheels: only while ENABLED and the drive stick is fresh
  if (!enabled || since_stick > kJoystickTimeout)
    stop_motors();
  else
    apply_stick(stick);

  // Seat motors: only while IDLE and their own stream is fresh; 0 otherwise
  target4_ =
      (!enabled && now - seat_stamp < kJoystickTimeout) ? std::clamp(seat.y, -1.0f, 1.0f) : 0.0f;
  target5_ =
      (!enabled && now - foot_stamp < kJoystickTimeout) ? std::clamp(foot.y, -1.0f, 1.0f) : 0.0f;

  update_motors();

  // Status: at once when the state changes (the HMI waits only ~750 ms for an
  // answer), otherwise every 500 ms by the clock, not by loop count.
  static State last_sent = State::IDLE;
  static auto last_status = std::chrono::steady_clock::time_point{};
  const State now_state = state_;
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

  // Inverse kinematics: (x, y, twist) -> each wheel's speed along its drive direction.
  const float target1 =
      -sample.x * std::cos(alpha1) + sample.y * std::sin(alpha1) - R * sample.twist;
  const float target2 =
      -sample.x * std::cos(alpha2) + sample.y * std::sin(alpha2) - R * sample.twist;
  const float target3 =
      -sample.x * std::cos(alpha3) + sample.y * std::sin(alpha3) - R * sample.twist;

  // Scale all three together so none exceeds 1: clamping each on its own would change the
  // direction the chair moves.
  const float biggest =
      std::max({std::fabs(target1), std::fabs(target2), std::fabs(target3), 1.0f});
  set_targets(target1 / biggest, target2 / biggest, target3 / biggest);
}
