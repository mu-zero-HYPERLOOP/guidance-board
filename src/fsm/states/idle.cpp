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

  pwm::disable_output();
  pwm::disable_trig0();
  pwm::disable_trig1();
  sdc_brake::open();
  sdc_brake::release_brake();

  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return guidance_state_IDLE;
}
