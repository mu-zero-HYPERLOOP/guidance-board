#pragma once

#include "canzero/canzero.h"

static void can_defaults() {
  canzero_set_assertion_fault(error_flag_OK);

  canzero_set_error_arming_failed(error_flag_OK);
  canzero_set_error_precharge_failed(error_flag_OK);
  canzero_set_error_heartbeat_miss(error_flag_OK);
  canzero_set_error_outer_airgap_left_invalid(error_flag_OK);
  canzero_set_error_inner_airgap_left_invalid(error_flag_OK);
  canzero_set_error_outer_airgap_right_invalid(error_flag_OK);
  canzero_set_error_inner_airgap_right_invalid(error_flag_OK);
  canzero_set_error_vdc_voltage_invalid(error_flag_OK);
  canzero_set_error_magnet_current_left_invalid(error_flag_OK);
  canzero_set_error_magnet_current_right_invalid(error_flag_OK);
  canzero_set_error_input_current_invalid(error_flag_OK);
  canzero_set_error_mcu_temperature_invalid(error_flag_OK);
  canzero_set_error_level_vdc_voltage(error_level_OK);
  canzero_set_error_level_magnet_current_left(error_level_OK);
  canzero_set_error_level_magnet_current_right(error_level_OK);
  canzero_set_error_level_input_current(error_level_OK);
  canzero_set_error_level_mcu_temperature(error_level_OK);
  canzero_set_error_sdc_brake(error_flag_OK);
  canzero_set_error_magnet_current_left_unexpected(error_flag_OK);
  canzero_set_error_magnet_current_right_unexpected(error_flag_OK);

  canzero_set_last_node_missed(255);

  canzero_set_ignore_45v(bool_t_FALSE);

}
