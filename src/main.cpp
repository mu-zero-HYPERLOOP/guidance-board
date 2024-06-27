#include "adc_config.h"
#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "defaults.h"
#include "firmware/guidance_board.h"
#include "fsm/fsm.h"
#include "pwm_config.h"
#include "sdc_brake.h"
#include "sensors/input_current.h"
#include "sensors/magnet_temperatures.h"
#include "sensors/mcu_temperature.h"
#include "sensors/vdc.h"
#include "xbar_config.h"

int main() {
  guidance_board::delay(3_s);

  canzero_init();
  can_defaults();

  fsm::begin();

  // Hardware config
  guidance_board::begin();
  adc_config();
  pwm_config();
  xbar_config();

  // Sensors
  sensors::input_current::begin();
  sensors::mcu_temperature::begin();
  sensors::magnet_temperatures::begin();
  sensors::vdc::begin();
  
  // Calibration
  sensors::input_current::calibrate();
  sensors::mcu_temperature::calibrate();
  sensors::magnet_temperatures::calibrate();
  sensors::vdc::calibrate();

  sdc_brake::begin();

  adc_isr::begin();
  control::begin();


  fsm::finish_init(); // init -> idle
  while (true) {
    canzero_can0_poll();
    canzero_can1_poll();
 
    guidance_board::update();

    sensors::input_current::update();
    sensors::mcu_temperature::update();
    sensors::magnet_temperatures::update();
    sensors::vdc::update();

    sdc_brake::update();

    adc_isr::update();
    control::update();

    fsm::update();

    canzero_update_continue(canzero_get_time());
  }
}

