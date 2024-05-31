#include "fsm/error_handling.h"
#include "canzero/canzero.h"

guidance_command fsm::error_handling::approve(guidance_command cmd) {

  if (canzero_get_error_level_mcu_temperature() == error_level_ERROR) {
    return guidance_command_DISARM45;
  }

  error_level error_levels[] = {
      canzero_get_error_level_mcu_temperature(),
      canzero_get_error_level_magnet_temperature_left(),
      canzero_get_error_level_magnet_temperature_right(),
  };

  for (size_t i = 0; i < sizeof(error_levels) / sizeof(error_level); ++i) {
    if (error_levels[i] == error_level_ERROR) {
      return guidance_command_DISARM45;
    }
  }

  if (canzero_get_error_arming_failed() == error_flag_ERROR) {
    return guidance_command_DISARM45;
  }

  if (guidance_command_DISARM45 != cmd) {
    for (size_t i = 0; i < sizeof(error_levels) / sizeof(error_level); ++i) {
      if (error_levels[i] == error_level_WARNING) {
        return guidance_command_STOP;
      }
    }
  }


  return cmd;
}
