#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "pwm_brake.h"
#include "sdc_brake.h"

guidance_state fsm::states::control(guidance_command cmd, Duration time_since_last_transition) {

  if (guidance_command_DISARM45 == cmd) {
    return guidance_state_IDLE;
  }

  if (guidance_command_STOP == cmd) {
    return guidance_state_READY;
  }

  if (!pwm_brake::request_start()) {
    canzero_set_command(guidance_command_NONE);
    return guidance_state_IDLE;
  }
  if (!sdc_brake::request_close()) {
    canzero_set_command(guidance_command_NONE);
    return guidance_state_IDLE;
  }
  guidance_board::set_digital(ctrl_pin::precharge_start_32, false);
  guidance_board::set_digital(ctrl_pin::precharge_done_31, true);

  return guidance_state_CONTROL;
}
