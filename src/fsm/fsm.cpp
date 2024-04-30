#include "canzero/canzero.h"

#include "fsm/error_handling.hpp"
#include "fsm/states.hpp"
#include "util/timestamp.hpp"

Timestamp g_fsm_last_transition = Timestamp::now();

void fsm_init() {
  g_fsm_last_transition = Timestamp::now();
  canzero_set_state(mgu_state_INIT);
}

void fsm_next() {
    Timestamp now = Timestamp::now();
    Duration time_since_last_transition = now - g_fsm_last_transition;

    mgu_state state = canzero_get_state();
    mgu_command cmd =
        handle_errors(state, canzero_get_command(), time_since_last_transition);

    mgu_state next_state;
    switch(state) {
        case mgu_state_INIT:
            next_state = init_state_next(cmd, time_since_last_transition);
            break;
        case mgu_state_IDLE:
            next_state = idle_state_next(cmd, time_since_last_transition);
            break;
        case mgu_state_PRECHARGE:
            next_state = precharge_state_next(cmd, time_since_last_transition);
            break;
        case mgu_state_READY:
            next_state = ready_state_next(cmd, time_since_last_transition);
            break;
        case mgu_state_START:
            next_state = start_state_next(cmd, time_since_last_transition);
            break;
        case mgu_state_CONTROL:
            next_state = control_state_next(cmd, time_since_last_transition);
            break;
        case mgu_state_STOP:
            next_state = stop_state_next(cmd, time_since_last_transition);
            break;
    }

    if (next_state != state) {
        g_fsm_last_transition = now;
        canzero_set_state(next_state);
    }
}
