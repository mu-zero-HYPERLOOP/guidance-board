#pragma once

#include "canzero/canzero.h"
#include "util/timestamp.hpp"

mgu_command handle_errors(mgu_state state, mgu_command cmd,
                          Duration time_since_last_transition);
