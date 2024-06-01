
#include "firmware/pwm.h"

struct GuidancePwmControl {
  float left_l = 0.5; // range [0,1] // rename phases!
  float left_r = 0.5;
  float right_l = 0.5;
  float right_r = 0.5;
  constexpr operator PwmControl() const {
    return PwmControl{.duty20 = 0,
                      .duty22 = left_l,
                      .duty23 = 0,
                      .duty42 = left_r,
                      .duty31 = right_l,
                      .duty13 = right_r};
  }

  GuidancePwmControl() : left_l(0.5), left_r(0.5), right_l(0.5), right_r(0.5) {}
};
