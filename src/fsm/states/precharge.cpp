#include "canzero/canzero.h"
#include "control.h"
#include "feedthrough_mosfet.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

constexpr Duration MIN_PRECHARGE_TIME = 1_s;
constexpr Duration MAX_PRECHARGE_TIME = 5_s;
constexpr Voltage REQUIRED_VDC_VOLTAGE = 40_V;

guidance_state fsm::states::precharge(guidance_command cmd,
                                      Duration time_since_last_transition) {

  if (guidance_command_DISARM45 == cmd) {
    return guidance_state_DISARMING45;
  }

  if (time_since_last_transition > MIN_PRECHARGE_TIME){
    return guidance_state_READY;
  }

  if (time_since_last_transition > MAX_PRECHARGE_TIME) {
    return guidance_state_DISARMING45; // we might want to set a error flag
                                       // here.
  }

  pwm::control(GuidancePwmControl());
  pwm::enable_output();
  pwm::disable_trig1();

  if (!sdc_brake::request_close()) {
    canzero_set_command(guidance_command_DISARM45);
    return guidance_state_DISARMING45;
  }

  precharge_mosfet::close();
  feedthrough_mosfet::open();

  return guidance_state_PRECHARGE;
}
