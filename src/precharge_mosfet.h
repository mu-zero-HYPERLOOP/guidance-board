#pragma once

#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
namespace precharge_mosfet {


static inline void close(){
  guidance_board::set_digital(ctrl_pin::precharge_start_32, true);
  canzero_set_precharge_status(sdc_status_CLOSED);
}

static inline void open(){
  guidance_board::set_digital(ctrl_pin::precharge_start_32, false);
  canzero_set_precharge_status(sdc_status_OPEN);
}

}
