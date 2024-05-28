#include "sensors/mcu_temperature.h"
#include "avr/pgmspace.h"
#include "canzero/canzero.h"
#include "firmware/guidance_board.h"
#include "util/boxcar.h"
#include "util/interval.h"
#include "util/metrics.h"

static DMAMEM BoxcarFilter<Temperature, 10> filter(24_Celcius);

static Interval interval(sensors::mcu_temperature::MEAS_FREQUENCY);

void sensors::mcu_temperature::begin(){
  // TODO set canzero vars.
}

void sensors::mcu_temperature::calibrate(){
  for (size_t i = 0; i < filter.size(); ++i) {
    filter.push(guidance_board::read_mcu_temperature());
    canzero_update_continue(canzero_get_time());
    guidance_board::delay(1_ms);
  }
}

void sensors::mcu_temperature::update() {
  if(interval.next()) {
    const Temperature temp = guidance_board::read_mcu_temperature();
    filter.push(temp);
  }
}
