#pragma once

#include "canzero/canzero.h"
#include "util/timestamp.h"

namespace fsm::states {

guidance_state init(guidance_command cmd, Duration time_since_last_transition);

guidance_state idle(guidance_command cmd, Duration time_since_last_transition);

guidance_state arming45(guidance_command cmd, Duration time_since_last_transition);

guidance_state precharge(guidance_command cmd, Duration time_since_last_transition);

guidance_state ready(guidance_command cmd, Duration time_since_last_transition);

guidance_state control(guidance_command cmd, Duration time_since_last_transition);

}
