#include "firmware/guidance_board.h"
#include "InternalTemperature.h"
#include "core_pins.h"
#include "firmware/ain_scheduler.h"
#include "firmware/xbar.h"
#include <Arduino.h>

constexpr size_t MAX_AIN_PERIODIC_JOBS = 10;

static AinScheduler<MAX_AIN_PERIODIC_JOBS> ain_scheduler;

void FLASHMEM guidance_board::begin() {
  pinMode(static_cast<uint8_t>(ctrl_pin::sdc_trig_37), OUTPUT);
  digitalWrite(static_cast<uint8_t>(ctrl_pin::sdc_trig_37), false);
  pinMode(static_cast<uint8_t>(ctrl_pin::precharge_done_31), OUTPUT);
  digitalWrite(static_cast<uint8_t>(ctrl_pin::precharge_done_31), false);
  pinMode(static_cast<uint8_t>(ctrl_pin::precharge_start_32), OUTPUT);
  digitalWrite(static_cast<uint8_t>(ctrl_pin::precharge_start_32), false);
  xbar::begin();
}

Voltage FASTRUN guidance_board::sync_read(ain_pin pin) {
  return adc_etc::read_single(pin);
}

Temperature guidance_board::read_mcu_temperature() {
  float temp = InternalTemperature.readTemperatureC();
  float temp_kelvin = temp - 273.15f;
  return Temperature(temp_kelvin);
}

bool FLASHMEM guidance_board::register_periodic_reading(
    const Time &period, ain_pin pin, void (*on_value)(const Voltage &v)) {
  return ain_scheduler.schedule(period, AinSchedulerJob{
                                            .pin = pin,
                                            .on_value = on_value,
                                        });
}

void FASTRUN guidance_board::set_digital(ctrl_pin pin, bool state) {
  digitalWrite(static_cast<uint8_t>(pin), state);
}

void guidance_board::delay(Duration delta) {
  delayMicroseconds(delta.as_us());
}

guidance_board::InterruptLock FASTRUN guidance_board::InterruptLock::acquire() {
  __disable_irq();
  return InterruptLock();
}

void FASTRUN guidance_board::InterruptLock::release() {
  if (m_acquired) {
    __enable_irq();
  }
  m_acquired = false;
}
FASTRUN guidance_board::InterruptLock::~InterruptLock() {
  if (m_acquired) {
    __enable_irq();
  }
  m_acquired = false;
}

void guidance_board::update() {
  ain_scheduler.update_continue();
}
