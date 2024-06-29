#pragma once

#include "canzero/canzero.h"

static void can_defaults() {
  canzero_set_assertion_fault(error_flag_OK);

  canzero_set_ignore_45v(bool_t_FALSE);

}
