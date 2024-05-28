
#include "firmware/pwm.h"

struct GuidancePwmControl {
  float left_l; // range [0,1] // rename phases!
  float left_r;
  float right_l;
  float right_r;
  constexpr operator PwmControl() const {
    return PwmControl{.duty20 = 0,
                      .duty22 = left_r,
                      .duty23 = 0,
                      .duty42 = left_l,
                      .duty31 = right_r,
                      .duty13 = right_l};
  }

  GuidancePwmControl() : left_l(0), left_r(0), right_l(0), right_r(0) {}
};
