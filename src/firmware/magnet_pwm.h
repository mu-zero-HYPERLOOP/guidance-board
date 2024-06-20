
#include "firmware/pwm.h"

struct GuidancePwmControl {
  float left_l = 0.5;  // pwm (22).
  float left_r = 0.5;  // pwm (42)
  float right_l = 0.5; // pwm (31)
  float right_r = 0.5; // pwm (13)
  constexpr operator PwmControl() const {
    return PwmControl(0.5f, left_l, 0.5f, left_r, right_l, right_r);
  }

  GuidancePwmControl() : left_l(0.5), left_r(0.5), right_l(0.5), right_r(0.5) {}
};
