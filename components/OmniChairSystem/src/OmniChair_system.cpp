#include "OmniChair_system.hpp"

#include <algorithm>
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
  if (!mcp1_->drive_both_duty(to_duty(target1_), to_duty(target2_), ec))
    logger_.error("MCP1 drive failed: {}", ec.message());
  if (!mcp2_->drive_duty(Axis::M1, to_duty(target3_), ec))
    logger_.error("MCP2 drive failed: {}", ec.message());
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

  if (std::chrono::steady_clock::now() - stamp > kJoystickTimeout)
    stop_motors();
  else
    apply_stick(stick);
  update_motors(); // every pass, so zeros really reach the RoboClaws

  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, 20ms);
  return false;
}

void OmniChairSystem::apply_stick(const rammp::XYTwist &sample) {
  logger_.info("Joystick x={:.2f} y={:.2f} twist={:.2f} buttons={}", sample.x, sample.y,
               sample.twist, static_cast<uint32_t>(sample.buttons));

  // wheel kinematics
  target1 = -1 * sample.x * cos(alpha1) + sample.y * sin(alpha1) - R * sample.twist;
  target2 = -1 * sample.x * cos(alpha2) + sample.y * sin(alpha2) - R * sample.twist;
  target3 = -1 * sample.x * cos(alpha3) + sample.y * sin(alpha3) - R * sample.twist;

  // TODO: omni kinematics (x, y, twist) -> three wheel targets
  set_targets(0.0f, 0.0f, 0.0f);
  // One-wheel bench test, wheels off the ground: set_targets(sample.y, 0.0f, 0.0f);
}
