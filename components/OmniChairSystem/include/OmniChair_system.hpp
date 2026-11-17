#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>

#include "base_component.hpp"
#include "esp32-p4-eth.hpp"
#include "messages.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"

class OmniChairSystem : public espp::BaseComponent {
public:
  OmniChairSystem();
  /// Bring up Ethernet, RTPS and the joystick subscriber, then start the control task.
  bool start();

private:
  bool init_ethernet();
  bool init_rtps();
  bool init_pubsub();
  void handle_joystick_message(const rammp::XYTwist &sample);
  bool control_loop(std::mutex &m, std::condition_variable &cv);
  void apply_stick(const rammp::XYTwist &sample);
  void stop_motors();

  static constexpr const char *kIpAddress = "192.168.4.1";
  static constexpr std::chrono::milliseconds kJoystickTimeout{200};

  std::mutex joystick_mutex_;
  rammp::XYTwist latest_joystick_{};
  std::chrono::steady_clock::time_point last_joystick_time_{};

  // Destroyed bottom-up: task, then subscriber, then the participant they use.
  std::unique_ptr<espp::RtpsParticipant> rtps_participant_;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_;
  std::unique_ptr<espp::Task> control_task_;
};