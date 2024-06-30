#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

guidance_state fsm::states::control(guidance_command cmd, Duration time_since_last_transition) {

  if (guidance_command_DISARM45 == cmd) {
    return guidance_state_DISARMING45;
  }

  if (guidance_command_STOP == cmd) {
    return guidance_state_READY;
  }

  pwm::enable_output();
  // pwm control set by isr!
  pwm::enable_trig1();

  if (!sdc_brake::request_close()) {
    canzero_set_command(guidance_command_NONE);
    return guidance_state_IDLE;
  }
  precharge_mosfet::open();
  feedthrough_mosfet::close();

  return guidance_state_CONTROL;
}
