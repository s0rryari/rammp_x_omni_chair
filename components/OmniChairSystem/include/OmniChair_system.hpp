#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <numbers>

#include "base_component.hpp"
#include "basicmicro.hpp"
#include "driver/uart.h"
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
  bool init_mcps();
  std::unique_ptr<espp::Basicmicro> make_mcp(uart_port_t port, int tx_pin, int rx_pin);
  void set_targets(float target1, float target2, float target3);
  void update_motors();
  int16_t to_duty(float target) const;
  bool init_ethernet();
  bool init_rtps();
  bool init_pubsub();
  void handle_joystick_message(const rammp::XYTwist &sample);
  bool control_loop(std::mutex &m, std::condition_variable &cv);
  void apply_stick(const rammp::XYTwist &sample);
  void stop_motors();

  // RoboClaw wiring (S1 = controller RX <- our TX, S2 = controller TX -> our RX)
  static constexpr int kUartBaud = 115200;
  static constexpr uart_port_t kMcp1Port = UART_NUM_1;
  static constexpr int kMcp1TxPin = 3;
  static constexpr int kMcp1RxPin = 2;
  static constexpr uart_port_t kMcp2Port = UART_NUM_2;
  static constexpr int kMcp2TxPin = 5;
  static constexpr int kMcp2RxPin = 4;
  /// Share of full power a target of 1.0 reaches. Start low; raise once it drives safely.
  static constexpr float kMaxDuty = 0.4f;

  static constexpr const char *kIpAddress = "192.168.4.1";
  static constexpr std::chrono::milliseconds kJoystickTimeout{200};

  // wheel kinematics
  //  wheel angle convention used in the kinematics formula
  static constexpr float alpha1 = 0.0f;                                    // front
  static constexpr float alpha2 = 4.0f * std::numbers::pi_v<float> / 3.0f; // rear left
  static constexpr float alpha3 = 2.0f * std::numbers::pi_v<float> / 3.0f; // rear right
  static constexpr float R = 1.0f;                                         // og was .35

  // Wheel targets, -1..+1. Only the control task touches these, so no mutex.
  float target1_{0.0f};
  float target2_{0.0f};
  float target3_{0.0f};

  std::mutex joystick_mutex_;
  rammp::XYTwist latest_joystick_{};
  std::chrono::steady_clock::time_point last_joystick_time_{};

  // Destroyed bottom-up: the task first, then what it uses.
  std::unique_ptr<espp::Basicmicro> mcp1_;
  std::unique_ptr<espp::Basicmicro> mcp2_;
  std::unique_ptr<espp::RtpsParticipant> rtps_participant_;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_;
  std::unique_ptr<espp::Task> control_task_;
};