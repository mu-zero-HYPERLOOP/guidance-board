#include "canzero/canzero.h"
#include "util/timestamp.hpp"

mgu_state idle_state_next(mgu_command cmd,
                            Duration time_since_last_transition) {

    if (mgu_command_PRECHARGE == cmd) {
        return mgu_state_PRECHARGE;
    } else {
        //Actions
        return mgu_state_IDLE;
    }
}
