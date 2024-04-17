#include "canzero.h"
#include "states.h"

mgu_state ready_state_next(mgu_command cmd,
                            Duration time_since_last_transition) {

    if (cmd == "TODO") {
        return mgu_state_START;
    } else {
        return mgu_state_READY;
    }
}
