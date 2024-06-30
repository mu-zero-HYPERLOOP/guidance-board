#include "canzero/canzero.h"
#include "feedthrough_mosfet.h"
#include "fsm/states.h"
#include "firmware/pwm.h"
#include "precharge_mosfet.h"
#include "sdc_brake.h"

constexpr Duration MIN_DISARM_TIME = 1_s;

guidance_state fsm::states::disarming45(guidance_command cmd, Duration time_since_last_transition){

  
  if (time_since_last_transition > MIN_DISARM_TIME){
    return guidance_state_IDLE;
  }


  pwm::control(PwmControl());
  pwm::enable_output();
  pwm::disable_trig1();

  sdc_brake::open();
  precharge_mosfet::open();
  feedthrough_mosfet::open();

  return guidance_state_DISARMING45;
}
