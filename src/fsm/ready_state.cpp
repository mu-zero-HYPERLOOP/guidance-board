#include "canzero/canzero.h"
#include "fsm/states.hpp"

mgu_state ready_state_next(mgu_command cmd,
                            Duration time_since_last_transition) {

  return mgu_state_READY;
}
