#pragma once

#include <atomic>
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

  // filtering functions
  float filter(float rawValue, float w, float prevValue);
  float selectFilter(float target, float previous);
  float chooseComp(float pos, float neg, float value);
  float limitAcceleration(float previous, float target, float accelLimit, float decelLimit);
  float applyDeadband(float w, float minDuty);
  // RoboClaw wiring (S1 = controller RX <- our TX, S2 = controller TX -> our RX)
  static constexpr int kUartBaud = 115200;
  static constexpr uart_port_t kMcp1Port = UART_NUM_1;
  static constexpr int kMcp1TxPin = 3;
  static constexpr int kMcp1RxPin = 2;
  static constexpr uart_port_t kMcp2Port = UART_NUM_2;
  static constexpr int kMcp2TxPin = 5;
  static constexpr int kMcp2RxPin = 4;
  /// Share of full power a target of 1.0 reaches. Start low; raise once it drives safely.
  static constexpr float kMaxDuty = 0.9f;

  static constexpr const char *kIpAddress = "192.168.4.1";
  static constexpr std::chrono::milliseconds kJoystickTimeout{200};
  static constexpr std::chrono::milliseconds kJoystickLostTimeout{1000};
  enum class State : uint8_t { IDLE, ENABLED };

  // wheel kinematics
  //  wheel angle convention used in the kinematics formula
  static constexpr float alpha1 = 0.0f;                                    // front
  static constexpr float alpha2 = 4.0f * std::numbers::pi_v<float> / 3.0f; // rear left
  static constexpr float alpha3 = 2.0f * std::numbers::pi_v<float> / 3.0f; // rear right
  static constexpr float R = 1.0f;

  // wheelchair mode limits
  const float max_turn = std::numbers::pi_v<float> * 0.3;
  const float max_vx_wc = 0.85;
  const float max_vy = 0.95;

  // omni mode limits
  const float max_vx_omni = 0.75;
  const float max_omega = std::numbers::pi_v<float> * 0.40;

  // magnitude cap for combined vx + vy motion
  const float max_v_mag = 0.95;

  // turning accel/decel boost so omega ramps as fast as linear motion
  const float OMEGA_ACCEL_BOOST = 1.0;

  // per-wheel direction-specific compensation.
  // strafe LEFT and RIGHT symmetrized (averaged) to match Square.txt.
  // previous asymmetric values are noted in the comments in case a rollback
  // is needed. structure kept in place so they can be split again later.
  const float COMP_W1_FWD = 1.0;
  const float COMP_W1_STRAFE_LEFT = 0.945;  // was 0.95
  const float COMP_W1_STRAFE_RIGHT = 0.945; // was 0.94
  const float COMP_W2_POS = 1.0;
  const float COMP_W2_NEG = 1.0;
  const float COMP_W2_STRAFE_LEFT = 1.05;  // was 1.08
  const float COMP_W2_STRAFE_RIGHT = 1.05; // was 1.025
  const float COMP_W3_POS = 1.0;
  const float COMP_W3_NEG = 1.0;
  const float COMP_W3_STRAFE_LEFT = 1.04;  // was 1.10
  const float COMP_W3_STRAFE_RIGHT = 1.04; // was 0.975

  // per-wheel minimum-duty deadband compensation.
  // may not be needed
  const float W1_MIN_DUTY = 0.0;
  const float W2_MIN_DUTY = 0.0;
  const float W3_MIN_DUTY = 0.0;

  // acceleration limiting (in units per second).
  const float a_max_accel = 4.5;
  const float a_max_decel = 4.5;
  const float a_hard_decel = 5.5;
  const float a_max_reversal = 8.0;
  const float a_start_accel = 6.0;
  const float START_ACCEL_RAMP_VEL = 0.2;

  // loop time conversion. accel values above are in units/sec.
  const float loop_dt = 0.02f;
  const float max_accel = a_max_accel * loop_dt;
  const float max_decel = a_max_decel * loop_dt;
  const float hard_decel = a_hard_decel * loop_dt;
  const float max_reversal = a_max_reversal * loop_dt;
  const float start_accel = a_start_accel * loop_dt;

  double vxfilter = 0.0, vyfilter = 0.0, omegafilter = 0.0;
  double pre_vxfilter = 0.0, pre_vyfilter = 0.0, pre_omegafilter = 0.0;

  float raw_w1 = 0.0, raw_w2 = 0.0, raw_w3 = 0.0;
  float wheel1 = 0.0, wheel2 = 0.0, wheel3 = 0.0;
  float prevWheel1 = 0.0, prevWheel2 = 0.0, prevWheel3 = 0.0;

  double Kfvel_accel = 1.0;
  double Kfvel_decel = 1.0;

  // Wheel targets, -1..+1. Only the control task touches these, so no mutex.
  float target1_{0.0f};
  float target2_{0.0f};
  float target3_{0.0f};

  std::mutex joystick_mutex_;
  rammp::XYTwist latest_joystick_{};
  std::chrono::steady_clock::time_point last_joystick_time_{};
  std::atomic<State> state_{State::IDLE};
  std::atomic<rammp::DriveProfile> profile_{rammp::DriveProfile::NORMAL};
  uint8_t status_seq_{0};

  // Destroyed bottom-up: the task first, then what it uses.
  std::unique_ptr<espp::Basicmicro> mcp1_;
  std::unique_ptr<espp::Basicmicro> mcp2_;
  std::unique_ptr<espp::RtpsParticipant> rtps_participant_;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_;
  std::unique_ptr<espp::Publisher<rammp::MibStatus>> status_pub_;
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_sub_;
  std::unique_ptr<espp::Task> control_task_;
};