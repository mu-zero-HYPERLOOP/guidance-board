
#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

guidance_state fsm::states::ready(guidance_command cmd, Duration time_since_last_transition) {

  if (guidance_command_DISARM45 == cmd) {
    return guidance_state_IDLE;
  }

  if (guidance_command_START == cmd) {
    return guidance_state_CONTROL;
  }

  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()) {
    canzero_set_command(guidance_command_NONE);
    return guidance_state_IDLE;
  }
  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return guidance_state_READY;
}
