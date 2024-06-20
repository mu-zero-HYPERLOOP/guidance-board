
#include "canzero/canzero.h"
#include <cstdio>
#include <cinttypes>


void canzero_can0_wdg_timeout(uint8_t node_id) {
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
  perror("heartbeat miss can0 detected");
}

void canzero_can1_wdg_timeout(uint8_t node_id) {
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
  perror("heartbeat miss can1 detected");
}
