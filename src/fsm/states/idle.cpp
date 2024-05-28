#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "pwm_brake.h"
#include "sdc_brake.h"

guidance_state fsm::states::idle(guidance_command cmd, Duration time_since_last_transition) {

  if (guidance_command_ARM45 == cmd) {
    return guidance_state_ARMING45;
  }

  pwm_brake::stop();
  pwm_brake::release_brake();
  sdc_brake::open();
  sdc_brake::release_brake();
  guidance_board::set_digital(ctrl_pin::precharge_start_32, false);
  guidance_board::set_digital(ctrl_pin::precharge_done_31, false);

  return guidance_state_IDLE;
}
