#include "canzero/canzero.h"
#include <algorithm>
#include <array>
#include <cinttypes>
#include "print.h"

static std::array<bool, node_id_count + MAX_DYN_HEARTBEATS> miss_can0;
static std::array<bool, node_id_count + MAX_DYN_HEARTBEATS> miss_can1;

void check_full_recover() {
  if (!std::any_of(miss_can0.begin(), miss_can0.end(), [](auto m) { return m; }) &&
      !std::any_of(miss_can1.begin(), miss_can1.end(), [](auto m) { return m; })) {
    canzero_set_error_heartbeat_miss(error_flag_OK);
    debugPrintf("full heartbeat recover\n");
  }
}

void canzero_can0_wdg_timeout(uint8_t node_id) {
  debugPrintf("CAN0 Heartbeat Miss %u\n", node_id);
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
  miss_can0[node_id] = true;
  debugPrintf("can0 timeout\n");
}

void canzero_can1_wdg_timeout(uint8_t node_id) {
  debugPrintf("CAN1 Heartbeat Miss %u\n", node_id);
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
  miss_can1[node_id] = true;
  debugPrintf("can1 timeout\n");
}

void canzero_can0_wdg_recovered(uint8_t node_id) {
  miss_can0[node_id] = false;
  debugPrintf("can0 %d recovered\n", node_id);
  check_full_recover();
}

void canzero_can1_wdg_recovered(uint8_t node_id) {
  miss_can1[node_id] = false;
  debugPrintf("can1 %d recovered\n", node_id);
  check_full_recover();
}

