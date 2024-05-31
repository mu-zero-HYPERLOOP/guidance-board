#include "canzero/canzero.h"
#include "fsm/states.h"
#include "firmware/guidance_board.h"
#include "pwm_brake.h"
#include "sdc_brake.h"

guidance_state fsm::states::init(guidance_command cmd, Duration time_since_last_transition) {
  canzero_set_error_arming_failed(error_flag_OK);
  pwm_brake::stop();
  sdc_brake::open();
  guidance_board::set_digital(ctrl_pin::precharge_start_32, false);
  guidance_board::set_digital(ctrl_pin::precharge_done_31, false);
  return guidance_state_IDLE;
}
