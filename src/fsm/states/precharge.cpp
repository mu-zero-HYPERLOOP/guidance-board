#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "fsm/states.h"
#include "pwm_brake.h"
#include "sdc_brake.h"

constexpr Duration MIN_PRECHARGE_TIME = 1_s;
constexpr Duration MAX_PRECHARGE_TIME = 5_s;
constexpr Voltage REQUIRED_VDC_VOLTAGE = 40_V;

guidance_state fsm::states::precharge(guidance_command cmd, Duration time_since_last_transition) {
  
  if (guidance_command_DISARM45 == cmd) {
    return guidance_state_IDLE;
  }

  if (time_since_last_transition > MIN_PRECHARGE_TIME && canzero_get_vdc_voltage() > static_cast<float>(REQUIRED_VDC_VOLTAGE)) {
    return guidance_state_READY;
  }

  if (time_since_last_transition > MAX_PRECHARGE_TIME) {
    return guidance_state_IDLE; // we might want to set a error flag here.
  }

  pwm_brake::stop();
  if (!sdc_brake::request_close()) {
    canzero_set_command(guidance_command_NONE);
    return guidance_state_IDLE;
  }
  guidance_board::set_digital(ctrl_pin::precharge_start_32, true);
  guidance_board::set_digital(ctrl_pin::precharge_done_31, false);

  return guidance_state_PRECHARGE;
}
