#include "canzero/canzero.h"
#include "control.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "firmware/guidance_board.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

guidance_state fsm::states::init(guidance_command cmd, Duration time_since_last_transition) {
  canzero_set_error_arming_failed(error_flag_OK);

  pwm::control(GuidancePwmControl());
  pwm::disable_output();
  pwm::disable_trig1();

  sdc_brake::open();

  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return guidance_state_IDLE;
}
