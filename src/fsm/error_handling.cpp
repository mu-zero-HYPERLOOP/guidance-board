#include "fsm/error_handling.hpp"
#include "canzero/canzero.h"

mgu_command handle_errors(mgu_state state, mgu_command cmd,
                             Duration time_since_last_transition) {

    return cmd;
}