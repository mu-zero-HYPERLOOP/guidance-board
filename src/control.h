#pragma once

#include "util/metrics.h"
#include "firmware/magnet_pwm.h"
namespace control {

void begin();

GuidancePwmControl control_loop(Current current_left, 
    Current current_right, Distance magnet_airgap_left,
    Distance lim_airgap_left,
    Distance magnet_airgap_right, 
    Distance lim_airgap_right);

void update();

} // namespace control
