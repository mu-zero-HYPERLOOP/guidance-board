#pragma once

#include "firmware/adc_etc.h"
#include "firmware/pinout.h"
#include "firmware/pwm.h"
#include "util/timestamp.h"
namespace guidance_board {


void begin();

Voltage sync_read(ain_pin);

Temperature read_mcu_temperature();

bool register_periodic_reading(const Time &period, ain_pin pin,
                               void (*on_value)(const Voltage &v));

inline bool register_periodic_reading(const Frequency &frequency, ain_pin pin,
                                      void (*on_value)(const Voltage &v)) {
  return register_periodic_reading(1.0f / frequency, pin, on_value);
}

void set_digital(ctrl_pin pin, bool state);

void delay(Duration delta);


struct InterruptLock {
public:
  static InterruptLock acquire();
  void release();
  ~InterruptLock();
  InterruptLock(const InterruptLock&) = delete;
  InterruptLock(InterruptLock&&) = delete;
  InterruptLock &operator=(const InterruptLock&) = delete;
  InterruptLock &operator=(InterruptLock&&) = delete;
private:
  InterruptLock() : m_acquired(true) {}
  bool m_acquired;
};

void update();

};
