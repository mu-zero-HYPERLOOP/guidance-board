#pragma once

#include "canzero/canzero.h"

static void can_defaults() {
  canzero_set_assertion_fault(error_flag_OK);
  canzero_set_error_heartbeat_miss(error_flag_OK);
  canzero_set_last_node_missed(255);
  canzero_set_error_input_current_invalid(error_flag_OK);

  canzero_set_ignore_45v(bool_t_FALSE);

}
