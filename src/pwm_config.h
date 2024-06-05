#pragma once

#include "util/metrics.h"
#include "firmware/pwm.h"


static void pwm_config() {
  PwmBeginInfo pwmBeginInfo;
  pwmBeginInfo.enable_outputs = false;
  pwmBeginInfo.frequency = 20_kHz;
  pwmBeginInfo.deadtime = 1000_ns;
  pwmBeginInfo.trig0 = 0.0f;
  pwmBeginInfo.enable_trig0 = false;
  pwmBeginInfo.enable_trig0_interrupt = false;
  pwmBeginInfo.trig1 = std::nullopt;
  pwmBeginInfo.enable_trig1 = false;
  pwmBeginInfo.enable_trig1_interrupt = false;
  pwm::begin(pwmBeginInfo);
}
