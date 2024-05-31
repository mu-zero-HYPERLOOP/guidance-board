#pragma once
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "pwm_brake.h"
#include "sdc_brake.h"
#include <iostream> 

constexpr Duration STATE_TIMEOUT = 5_s;

guidance_state fsm::states::arming45(guidance_command cmd, Duration time_since_last_transition) {
  
  if (guidance_command_DISARM45 == cmd) {
    return guidance_state_IDLE;
  }

  if (guidance_command_PRECHARGE == cmd) {
    return guidance_state_PRECHARGE;
  }


  pwm_brake::stop();
  if (!sdc_brake::request_close()) {
    canzero_set_command(guidance_command_NONE);
    return guidance_state_IDLE;
  }

  if (time_since_last_transition > STATE_TIMEOUT) {
    canzero_set_error_arming_failed(error_flag_ERROR);
    return guidance_state_ARMING45;
  }

  guidance_board::set_digital(ctrl_pin::sdc_trig_37, true);
  guidance_board::set_digital(ctrl_pin::precharge_start_32, false);
  guidance_board::set_digital(ctrl_pin::precharge_done_31, false);

  return guidance_state_ARMING45;
}

