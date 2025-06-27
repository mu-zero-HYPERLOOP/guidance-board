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
#include "sensors/magnet_current.h"
#include "sensors/magnet_temperatures.h"
#include "sensors/mcu_temperature.h"
#include "sensors/vdc.h"
#include "xbar_config.h"
#include "print.h"
#include "sensors/airgaps.h"
#include <chrono>
#include <thread>

int main() {
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
  sensors::airgaps::begin();
  sensors::magnet_current::begin();
  
  // Calibration
  sensors::input_current::calibrate();
  sensors::mcu_temperature::calibrate();
  sensors::magnet_temperatures::calibrate();
  sensors::vdc::calibrate();
  sensors::airgaps::calibrate();
  sensors::magnet_current::begin();

  sdc_brake::begin();

  adc_isr::begin();
  control::begin();

  fsm::finish_init(); // init -> idle

  // 1. Go to ARMING45
  canzero_set_command(guidance_command_ARM45);
  for (int i = 0; i < 10; ++i) { fsm::update(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

  // 2. Go to PRECHARGE
  canzero_set_command(guidance_command_PRECHARGE);
  for (int i = 0; i < 10; ++i) { fsm::update(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

  // 3. Wait for precharge to complete (simulate time passing)
  for (int i = 0; i < 100; ++i) { fsm::update(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

  // 4. Go to CONTROL
  canzero_set_command(guidance_command_START);
  for (int i = 0; i < 10; ++i) { fsm::update(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

  // Now enter the main loop
  while (true) {
    canzero_can0_poll();
    canzero_can1_poll();
 
    guidance_board::update();

    sensors::input_current::update();
    sensors::mcu_temperature::update();
    sensors::magnet_temperatures::update();
    sensors::vdc::update();
    sensors::airgaps::update();
    sensors::magnet_current::update();

    sdc_brake::update();

    adc_isr::update();
    control::update();

    fsm::update();

    canzero_update_continue(canzero_get_time());
  }
}

