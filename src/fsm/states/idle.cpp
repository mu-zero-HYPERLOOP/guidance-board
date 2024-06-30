#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

guidance_state fsm::states::idle(guidance_command cmd, Duration time_since_last_transition) {

  if (guidance_command_ARM45 == cmd) {
    return guidance_state_ARMING45;
  }

  if (time_since_last_transition > 3_s){
    canzero_set_error_arming_failed(error_flag_OK);
    canzero_set_error_precharge_failed(error_flag_OK);
  }

  sdc_brake::open();
  sdc_brake::release_brake();

  pwm::disable_output();
  pwm::disable_trig1();

  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return guidance_state_IDLE;
}
