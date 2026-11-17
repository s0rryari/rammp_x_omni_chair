#include <chrono>
#include <thread>

#include "driver/uart.h"

#include "basicmicro.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

// UART wiring for the MCP: it defaults to packet serial mode at 38400 baud; S1
// (controller RX) goes to our TX pin and S2 (controller TX) goes to our RX pin.
// File-scope so the captureless transport lambdas below can reference them
// without any capture-semantics ambiguity.
static constexpr uart_port_t uart_port_1 = UART_NUM_1;
static constexpr uart_port_t uart_port_2 = UART_NUM_2;
static constexpr int uart1_tx_pin = 3; // -> MCP S1
static constexpr int uart1_rx_pin = 2; // <- MCP S2
static constexpr int uart2_tx_pin = 5; // -> MCP S1
static constexpr int uart2_rx_pin = 4; // <- MCP S2
static constexpr int uart_baud = 115200;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Basicmicro Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting basicmicro example");

  //! [basicmicro example]

  uart_config_t uart_config = {};
  uart_config.baud_rate = uart_baud;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(uart_port_1, 256, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_driver_install(uart_port_2, 256, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_port_1, &uart_config));
  ESP_ERROR_CHECK(uart_param_config(uart_port_2, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_port_1, uart1_tx_pin, uart1_rx_pin, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_pin(uart_port_2, uart2_tx_pin, uart2_rx_pin, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  espp::Basicmicro mcp1({
      .address = 0x80, // default packet serial address (0x80 - 0x87)
      .write =
          [](std::span<const uint8_t> data) {
            uart_flush_input(uart_port_1); // drop any late / leftover reply bytes
            const int written = uart_write_bytes(
                uart_port_1, reinterpret_cast<const char *>(data.data()), data.size());
            return written == static_cast<int>(data.size());
          },

      .read = [](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
        const int read =
            uart_read_bytes(uart_port_1, data.data(), data.size(), pdMS_TO_TICKS(timeout.count()));
        return read < 0 ? 0 : static_cast<size_t>(read);
      },
      // must be >= 10 ms: a 10 ms quiet gap is also what clears the
      // controller's packet buffer after a communication error
      .timeout = 100ms,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  espp::Basicmicro mcp2({
      .address = 0x80, // default packet serial address (0x80 - 0x87)
      .write =
          [](std::span<const uint8_t> data) {
            uart_flush_input(uart_port_2); // drop any late / leftover reply bytes
            const int written = uart_write_bytes(
                uart_port_2, reinterpret_cast<const char *>(data.data()), data.size());
            return written == static_cast<int>(data.size());
          },

      .read = [](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
        const int read =
            uart_read_bytes(uart_port_2, data.data(), data.size(), pdMS_TO_TICKS(timeout.count()));
        return read < 0 ? 0 : static_cast<size_t>(read);
      },
      // must be >= 10 ms: a 10 ms quiet gap is also what clears the
      // controller's packet buffer after a communication error
      .timeout = 100ms,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;

  // identify the controller
  std::string version;
  if (mcp1.read_firmware_version(version, ec)) {
    logger.info("MCP1 Firmware version: '{}'", version);
  } else {
    logger.error("Could not read firmware version: {}", ec.message());
    logger.error("Is the controller connected, powered, and in packet serial mode?");
  }

  if (mcp2.read_firmware_version(version, ec)) {
    logger.info("MCP2 Firmware version: '{}'", version);
  } else {
    logger.error("Could not read firmware version: {}", ec.message());
    logger.error("Is the controller connected, powered, and in packet serial mode?");
  }

  // float volts{0};
  // if (mcp.read_main_battery_voltage(volts, ec))
  //   logger.info("Main battery: {:.1f} V", volts);

  // uint32_t status{0};
  // if (mcp.read_status(status, ec))
  //   logger.info("Status: 0x{:08X}{}", status, status == 0 ? " (normal)" : "");

  // // gentle speed ramp on M1 (up to ~12.5% duty) with encoder readback, then
  // // back down to a stop. Duty-cycle drive works without a tuned velocity PID;
  // // if your encoders + PID are configured, try drive_speed(Axis::M1, ...)
  // // instead. The channel is selected with the shared espp::MotorAxis enum.
  using Axis = espp::Basicmicro::Axis;
  static constexpr int16_t duty = 6553; // of 32767
  // command to drive duty: !mcp.drive_duty(Axis::M1, duty, ec)

  if (!mcp1.drive_duty(Axis::M1, duty, ec))
    logger.error("MCP1 drive_duty failed: {}", ec.message());
  if (!mcp2.drive_duty(Axis::M1, duty, ec))
    logger.error("MCP2 drive_duty failed: {}", ec.message());

  std::this_thread::sleep_for(3s);

  if (!mcp1.drive_duty(Axis::M1, 0, ec))
    logger.error("MCP1 stop failed: {}", ec.message());
  if (!mcp2.drive_duty(Axis::M1, 0, ec))
    logger.error("MCP2 stop failed: {}", ec.message());
}
