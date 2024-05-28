
#include "adc_config.h"
#include "adc_isr.h"
#include "canzero/canzero.h"
#include "control.h"
#include "firmware/guidance_board.h"
#include "fsm/fsm.h"
#include "pwm_brake.h"
#include "pwm_config.h"
#include "sdc_brake.h"
#include "sensors/magnet_temperatures.h"
#include "sensors/mcu_temperature.h"
#include "xbar_config.h"

int main() {
  canzero_init();

  fsm::begin();

  // Hardware config
  guidance_board::BeginInfo beginInfo;
  beginInfo.adcBeginInfo = adc_config();
  beginInfo.pwmBeginInfo = pwm_config();
  guidance_board::begin(beginInfo);
  xbar_config();

  // Sensors
  sensors::mcu_temperature::begin();
  sensors::magnet_temperatures::begin();
  
  // Calibration
  sensors::mcu_temperature::calibrate();
  sensors::magnet_temperatures::calibrate();

  pwm_brake::begin();
  sdc_brake::begin();

  control::begin();

  fsm::finish_init(); // init -> idle
  while (true) {
    canzero_can0_poll();
    canzero_can1_poll();
 
    guidance_board::update();

    sensors::mcu_temperature::update();
    sensors::magnet_temperatures::update();
    adc_isr::update();
    sdc_brake::update();
    control::update();
    fsm::update();

    canzero_update_continue(canzero_get_time());
  }
}

