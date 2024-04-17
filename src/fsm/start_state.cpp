#include "canzero.h"
#include "states.h"

mgu_state start_state_next(mgu_command cmd,
                            Duration time_since_last_transition) {
    return mgu_state_CONTROL;
}
