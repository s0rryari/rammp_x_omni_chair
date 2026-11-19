#include "OmniChair_system.hpp"
#include "logger.hpp"

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "OmniChair Main", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting OmniChair System");

  static OmniChairSystem chair; // static: keeps running after app_main returns
  if (!chair.start()) {
    logger.error("OmniChair failed to start (see the OmniChair log above)");
    return;
  }
  logger.info("OmniChair running");
}
