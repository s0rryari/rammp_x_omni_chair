#include "OmniChair_system.hpp"

#include <thread>

using namespace std::chrono_literals;

OmniChairSystem::OmniChairSystem()
    : BaseComponent("OmniChair", espp::Logger::Verbosity::INFO) {}

bool OmniChairSystem::start() {
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

  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, 20ms);
  return false; // keep running
}

void OmniChairSystem::apply_stick(const rammp::XYTwist &sample) {
  // TODO: omni kinematics -> wheel commands. For now, just show it arrives:
  logger_.info("Joystick x={:.2f} y={:.2f} twist={:.2f} buttons={}", sample.x, sample.y,
               sample.twist, static_cast<uint32_t>(sample.buttons));
}
//   using Axis = espp::Basicmicro::Axis;
//   static constexpr int16_t duty = 6553; // of 32767

//   if (!mcp1.drive_duty(Axis::M1, duty, ec))
//     logger.error("MCP1 drive_duty failed: {}", ec.message());
//   if (!mcp2.drive_duty(Axis::M1, duty, ec))
//     logger.error("MCP2 drive_duty failed: {}", ec.message());

//   std::this_thread::sleep_for(3s);

//   if (!mcp1.drive_duty(Axis::M1, 0, ec))
//     logger.error("MCP1 stop failed: {}", ec.message());
//   if (!mcp2.drive_duty(Axis::M1, 0, ec))
//     logger.error("MCP2 stop failed: {}", ec.message());

void OmniChairSystem::stop_motors() {
  // TODO: command all wheels to 0
}
