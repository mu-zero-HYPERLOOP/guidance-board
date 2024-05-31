#pragma once
#include "firmware/pwm.h"
#include "firmware/xbar.h"
#include "firmware/adc_etc.h"

static void xbar_config() {
  // Hook PWM trig signals to adc trig signals
  xbar::connect(pwm::TRIG0_SIGNAL_SOURCE, adc_etc::TRIG0_SIGNAL_SINK);
  xbar::connect(pwm::TRIG1_SIGNAL_SOURCE, adc_etc::TRIG1_SIGNAL_SINK);
}
