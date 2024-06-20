#include "fsm/error_handling.h"
#include "canzero/canzero.h"
#include <algorithm>
#include <array>

guidance_command fsm::error_handling::approve(guidance_command cmd) {

  const auto error_flags = std::array<error_flag, 3> {
    canzero_get_error_arming_failed(),
    canzero_get_error_heartbeat_miss(),
    canzero_get_error_precharge_failed(),
  };

  const auto max_error_flag_it = std::max_element(error_flags.begin(), error_flags.end());
  const error_flag max_error_flag = *max_error_flag_it;
  if (max_error_flag == error_flag_ERROR){
    // early bail out.
    return guidance_command_DISARM45;
  }

  const auto error_levels = std::array<error_level, 7> {
      canzero_get_error_level_mcu_temperature(),
      canzero_get_error_level_magnet_temperature_left(),
      canzero_get_error_level_magnet_temperature_right(),
      canzero_get_error_level_vdc_voltage(),
      canzero_get_error_level_input_current(),
      canzero_get_error_level_magnet_current_left(),
      canzero_get_error_level_magnet_current_right(),
  };

  const auto max_error_level_it = std::max_element(error_levels.begin(), error_levels.end());
  const error_level max_error_level = *max_error_level_it;

  switch (max_error_level) {
    case error_level_OK:
    case error_level_INFO:
      return cmd;
    case error_level_WARNING:
      return guidance_command_STOP;
    case error_level_ERROR:
      return guidance_command_DISARM45;
    default:
      return guidance_command_DISARM45;
  }
  return cmd;
}
