#pragma once

#include "canzero.h"
#include "timestamp.h"

mgu_state init_state_next(mgu_command cmd, Duration time_since_last_transition);

mgu_state idle_state_next(mgu_command cmd, Duration time_since_last_transition);

mgu_state precharge_state_next(mgu_command cmd, Duration time_since_last_transition);

mgu_state ready_state_next(mgu_command cmd, Duration time_since_last_transition);

mgu_state start_state_next(mgu_command cmd, Duration time_since_last_transition);

mgu_state control_state_next(mgu_command cmd, Duration time_since_last_transition);

mgu_state stop_state_next(mgu_command cmd, Duration time_since_last_transition);