#pragma once

#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
namespace feedthrough_mosfet {

static inline void close(){
  guidance_board::set_digital(ctrl_pin::precharge_done_31, true);
  canzero_set_feedthrough_status(sdc_status_CLOSED);
}

static inline void open(){
  guidance_board::set_digital(ctrl_pin::precharge_done_31, false);
  canzero_set_feedthrough_status(sdc_status_OPEN);
}

}
