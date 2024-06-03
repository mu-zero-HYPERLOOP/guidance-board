#pragma once

#include "canzero/canzero.h"
#include "firmware/guidance_board.h"


namespace sdc_brake {

extern volatile bool brake_engaged;
extern volatile sdc_status sdc_state;

inline void begin() {
  brake_engaged = false;
  guidance_board::set_digital(ctrl_pin::sdc_trig_37, false);
  sdc_state = sdc_status_OPEN;
}

// Immeditaly opens the SDC, this action
// cannot be reset with request_close,
// it requries release_brake to be called to
// close the SDC again, which only happens when
// the board is in the idle state.
static __attribute__((always_inline)) void brake_immediatly() {
  brake_engaged = true;
  guidance_board::set_digital(ctrl_pin::sdc_trig_37, false);
  sdc_state = sdc_status_OPEN;
}

static inline void release_brake() { brake_engaged = false; }

static inline bool request_close() {
  if (brake_engaged) {
    guidance_board::set_digital(ctrl_pin::sdc_trig_37, false);
    sdc_state = sdc_status_OPEN;
    return false;
  } else {
    guidance_board::set_digital(ctrl_pin::sdc_trig_37, true);
    sdc_state = sdc_status_CLOSED;
    return true;
  }
}

static inline void open() {
  guidance_board::set_digital(ctrl_pin::sdc_trig_37, false);
  sdc_state = sdc_status_OPEN;
}

inline void update(){
  canzero_set_sdc_status(sdc_state);
}

} // namespace sdc_brake
