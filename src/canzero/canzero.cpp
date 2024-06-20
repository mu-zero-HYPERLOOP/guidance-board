#include "canzero.h"
#include <avr/pgmspace.h>
uint32_t min_u32(uint32_t v, uint32_t max) {
    if (v > max) {
        return max;
    }
    return v;
}
uint64_t min_u64(uint64_t v, uint64_t max) {
    if (v > max) {
        return max;
    }
    return v;
}
uint64_t DMAMEM __oe_config_hash;
date_time DMAMEM __oe_build_time;
guidance_state DMAMEM __oe_state;
sdc_status DMAMEM __oe_sdc_status;
guidance_command DMAMEM __oe_command;
sdc_status DMAMEM __oe_precharge_status;
sdc_status DMAMEM __oe_feedthrough_status;
bool_t DMAMEM __oe_control_active;
error_flag DMAMEM __oe_error_arming_failed;
error_flag DMAMEM __oe_error_precharge_failed;
float DMAMEM __oe_outer_airgap_left;
float DMAMEM __oe_inner_airgap_left;
float DMAMEM __oe_outer_airgap_right;
float DMAMEM __oe_inner_airgap_right;
float DMAMEM __oe_vdc_voltage;
error_level DMAMEM __oe_error_level_vdc_voltage;
error_level_config DMAMEM __oe_error_level_config_vdc_voltage;
float DMAMEM __oe_current_left;
float DMAMEM __oe_current_right;
float DMAMEM __oe_input_current;
error_level_config DMAMEM __oe_error_level_config_magnet_current;
error_level DMAMEM __oe_error_level_magnet_current_left;
error_level DMAMEM __oe_error_level_magnet_current_right;
error_level_config DMAMEM __oe_error_level_config_input_current;
error_level DMAMEM __oe_error_level_input_current;
float DMAMEM __oe_magnet_temperature_left1;
float DMAMEM __oe_magnet_temperature_left2;
float DMAMEM __oe_magnet_temperature_left_max;
error_level DMAMEM __oe_error_level_magnet_temperature_left;
float DMAMEM __oe_magnet_temperature_right1;
float DMAMEM __oe_magnet_temperature_right2;
float DMAMEM __oe_magnet_temperature_right_max;
error_level DMAMEM __oe_error_level_magnet_temperature_right;
error_level_config DMAMEM __oe_error_level_config_magnet_temperature;
float DMAMEM __oe_mcu_temperature;
error_level DMAMEM __oe_error_level_mcu_temperature;
error_level_config DMAMEM __oe_error_level_config_mcu_temperature;
float DMAMEM __oe_loop_frequency;
error_flag DMAMEM __oe_assertion_fault;
error_flag DMAMEM __oe_error_heartbeat_miss;
float DMAMEM __oe_gamepad_lt2;
float DMAMEM __oe_gamepad_rt2;
float DMAMEM __oe_gamepad_lsb_x;
float DMAMEM __oe_gamepad_lsb_y;
float DMAMEM __oe_gamepad_rsb_x;
float DMAMEM __oe_gamepad_rsb_y;
bool_t DMAMEM __oe_gamepad_lt1_down;
bool_t DMAMEM __oe_gamepad_rt1_down;
bool_t DMAMEM __oe_gamepad_x_down;
static void canzero_serialize_canzero_message_get_resp(canzero_message_get_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x17D;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_header.m_sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->m_header.m_od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->m_header.m_client_id << 16;
  ((uint32_t*)data)[0] |= msg->m_header.m_server_id << 24;
  ((uint32_t*)data)[1] = msg->m_data;
}
static void canzero_serialize_canzero_message_set_resp(canzero_message_set_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x1BD;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->m_header.m_od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->m_header.m_client_id << 13;
  ((uint32_t*)data)[0] |= msg->m_header.m_server_id << 21;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_erno & (0xFF >> (8 - 1))) << 29;
}
static void canzero_serialize_canzero_message_guidance_board_front_stream_state(canzero_message_guidance_board_front_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xE9;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_sdc_status & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_command & (0xFF >> (8 - 3))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_control_active & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_precharge_status & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_feedthrough_status & (0xFF >> (8 - 1))) << 9;
}
static void canzero_serialize_canzero_message_guidance_board_front_stream_errors(canzero_message_guidance_board_front_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xBB;
  frame->dlc = 3;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_error_level_vdc_voltage & (0xFF >> (8 - 2)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_arming_failed & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_precharge_failed & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_level_magnet_current_left & (0xFF >> (8 - 2))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_level_magnet_current_right & (0xFF >> (8 - 2))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_level_input_current & (0xFF >> (8 - 2))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_level_magnet_temperature_left & (0xFF >> (8 - 2))) << 10;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_level_magnet_temperature_right & (0xFF >> (8 - 2))) << 12;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_level_mcu_temperature & (0xFF >> (8 - 2))) << 14;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_error_heartbeat_miss & (0xFF >> (8 - 1))) << 16;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_assertion_fault & (0xFF >> (8 - 1))) << 17;
}
static void canzero_serialize_canzero_message_guidance_board_front_stream_debug(canzero_message_guidance_board_front_stream_debug* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x13C;
  frame->dlc = 4;
  uint32_t loop_frequency_0 = ((msg->m_loop_frequency - 0) / 0.00000023283064370807974) + 0.5f;
  if (loop_frequency_0 > 0xFFFFFFFF) {
    loop_frequency_0 = 0xFFFFFFFF;
  }
  ((uint32_t*)data)[0] = loop_frequency_0;
}
static void canzero_serialize_canzero_message_guidance_board_front_stream_voltage_and_currents(canzero_message_guidance_board_front_stream_voltage_and_currents* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xFB;
  frame->dlc = 8;
  uint32_t vdc_voltage_0 = ((msg->m_vdc_voltage - 0) / 0.0015259021896696422) + 0.5f;
  if (vdc_voltage_0 > 0xFFFF) {
    vdc_voltage_0 = 0xFFFF;
  }
  ((uint32_t*)data)[0] = vdc_voltage_0;
  uint32_t current_left_16 = ((msg->m_current_left - -25) / 0.0007629510948348211) + 0.5f;
  if (current_left_16 > 0xFFFF) {
    current_left_16 = 0xFFFF;
  }
  ((uint32_t*)data)[0] |= current_left_16 << 16;
  uint32_t current_right_32 = ((msg->m_current_right - -25) / 0.0007629510948348211) + 0.5f;
  if (current_right_32 > 0xFFFF) {
    current_right_32 = 0xFFFF;
  }
  ((uint32_t*)data)[1] = current_right_32;
  uint32_t input_current_48 = ((msg->m_input_current - -25) / 0.0007629510948348211) + 0.5f;
  if (input_current_48 > 0xFFFF) {
    input_current_48 = 0xFFFF;
  }
  ((uint32_t*)data)[1] |= input_current_48 << 16;
}
static void canzero_serialize_canzero_message_guidance_board_front_stream_airgaps(canzero_message_guidance_board_front_stream_airgaps* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xFC;
  frame->dlc = 8;
  uint32_t outer_airgap_left_0 = ((msg->m_outer_airgap_left - 0) / 0.00030518043793392844) + 0.5f;
  if (outer_airgap_left_0 > 0xFFFF) {
    outer_airgap_left_0 = 0xFFFF;
  }
  ((uint32_t*)data)[0] = outer_airgap_left_0;
  uint32_t inner_airgap_left_16 = ((msg->m_inner_airgap_left - 0) / 0.00030518043793392844) + 0.5f;
  if (inner_airgap_left_16 > 0xFFFF) {
    inner_airgap_left_16 = 0xFFFF;
  }
  ((uint32_t*)data)[0] |= inner_airgap_left_16 << 16;
  uint32_t outer_airgap_right_32 = ((msg->m_outer_airgap_right - 0) / 0.00030518043793392844) + 0.5f;
  if (outer_airgap_right_32 > 0xFFFF) {
    outer_airgap_right_32 = 0xFFFF;
  }
  ((uint32_t*)data)[1] = outer_airgap_right_32;
  uint32_t inner_airgap_right_48 = ((msg->m_inner_airgap_right - 0) / 0.00030518043793392844) + 0.5f;
  if (inner_airgap_right_48 > 0xFFFF) {
    inner_airgap_right_48 = 0xFFFF;
  }
  ((uint32_t*)data)[1] |= inner_airgap_right_48 << 16;
}
static void canzero_serialize_canzero_message_heartbeat_can0(canzero_message_heartbeat_can0* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x1E5;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = msg->m_node_id;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_unregister & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_ticks_next & (0xFF >> (8 - 7))) << 9;
}
static void canzero_serialize_canzero_message_heartbeat_can1(canzero_message_heartbeat_can1* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x1E4;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = msg->m_node_id;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_unregister & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_ticks_next & (0xFF >> (8 - 7))) << 9;
}
static void canzero_deserialize_canzero_message_get_req(canzero_frame* frame, canzero_message_get_req* msg) {
  uint8_t* data = frame->data;
  msg->m_header.m_od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->m_header.m_client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->m_header.m_server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
}
static void canzero_deserialize_canzero_message_set_req(canzero_frame* frame, canzero_message_set_req* msg) {
  uint8_t* data = frame->data;
  msg->m_header.m_sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->m_header.m_eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_header.m_toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_header.m_od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->m_header.m_client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->m_header.m_server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->m_data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_gamepad_stream_input(canzero_frame* frame, canzero_message_gamepad_stream_input* msg) {
  uint8_t* data = frame->data;
  msg->m_lt2 = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0;
  msg->m_rt2 = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0;
  msg->m_lsb_x = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_lsb_y = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_rsb_x = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_rsb_y = ((((uint32_t*)data)[1] >> 8) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_lt1_down = (bool_t)((((uint32_t*)data)[1] >> 16) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_rt1_down = (bool_t)((((uint32_t*)data)[1] >> 17) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_x_down = (bool_t)((((uint32_t*)data)[1] >> 18) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_y_down = (bool_t)((((uint32_t*)data)[1] >> 19) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_b_down = (bool_t)((((uint32_t*)data)[1] >> 20) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_a_down = (bool_t)((((uint32_t*)data)[1] >> 21) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_lsb_down = (bool_t)((((uint32_t*)data)[1] >> 22) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_rsb_down = (bool_t)((((uint32_t*)data)[1] >> 23) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_heartbeat_can0(canzero_frame* frame, canzero_message_heartbeat_can0* msg) {
  uint8_t* data = frame->data;
  msg->m_node_id = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8)));
  msg->m_unregister = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_ticks_next = ((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 7)));
}
static void canzero_deserialize_canzero_message_heartbeat_can1(canzero_frame* frame, canzero_message_heartbeat_can1* msg) {
  uint8_t* data = frame->data;
  msg->m_node_id = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8)));
  msg->m_unregister = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_ticks_next = ((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 7)));
}
__attribute__((weak)) void canzero_can0_wdg_timeout(uint8_t node_id) {}
__attribute__((weak)) void canzero_can0_wdg_recovered(uint8_t node_id) {}
__attribute__((weak)) void canzero_can1_wdg_timeout(uint8_t node_id) {}
__attribute__((weak)) void canzero_can1_wdg_recovered(uint8_t node_id) {}

typedef enum {
  HEARTBEAT_JOB_TAG = 0,
  HEARTBEAT_WDG_JOB_TAG = 1,
  GET_RESP_FRAGMENTATION_JOB_TAG = 2,
  STREAM_INTERVAL_JOB_TAG = 3,
} job_tag;

typedef struct {
  uint32_t *buffer;
  uint8_t offset;
  uint8_t size;
  uint8_t od_index;
  uint8_t client_id;
} get_resp_fragmentation_job;

typedef struct {
  uint32_t last_schedule; 
  uint32_t stream_id;
} stream_interval_job;

typedef struct {
  unsigned int* can0_static_wdg_armed;
  int* can0_static_tick_countdowns;
  unsigned int* can0_dynamic_wdg_armed;
  int* can0_dynamic_tick_countdowns;

  unsigned int* can1_static_wdg_armed;
  int* can1_static_tick_countdowns;
  unsigned int* can1_dynamic_wdg_armed;
  int* can1_dynamic_tick_countdowns;
} heartbeat_wdg_job_t;

typedef struct {
  uint32_t climax;
  uint32_t position;
  job_tag tag;
  union {
    get_resp_fragmentation_job get_fragmentation_job;
    stream_interval_job stream_job;
    heartbeat_wdg_job_t wdg_job;
  } job;
} job_t;

union job_pool_allocator_entry {
  job_t job;
  union job_pool_allocator_entry *next;
};

typedef struct {
  union job_pool_allocator_entry job[64];
  union job_pool_allocator_entry *freelist;
} job_pool_allocator;

static job_pool_allocator DMAMEM job_allocator;
static void job_pool_allocator_init() {
  for (uint8_t i = 1; i < 64; i++) {
    job_allocator.job[i - 1].next = job_allocator.job + i;
  }
  job_allocator.job[64 - 1].next = NULL;
  job_allocator.freelist = job_allocator.job;
}

static job_t *job_pool_allocator_alloc() {
  if (job_allocator.freelist != NULL) {
    job_t *job = &job_allocator.freelist->job;
    job_allocator.freelist = job_allocator.freelist->next;
    return job;
  } else {
    return NULL;
  }
}

static void job_pool_allocator_free(job_t *job) {
  union job_pool_allocator_entry *entry = (union job_pool_allocator_entry *)job;
  entry->next = job_allocator.freelist;
  job_allocator.freelist = entry;
}

#define SCHEDULER_HEAP_SIZE 71
typedef struct {
  job_t *heap[SCHEDULER_HEAP_SIZE]; // job**
  uint32_t size;
} job_scheduler_t;

static job_scheduler_t DMAMEM scheduler;
static void scheduler_promote_job(job_t *job) {
  int index = job->position;
  if (index == 0) {
    return;
  }
  int parent = (job->position - 1) / 2;
  while (scheduler.heap[parent]->climax > scheduler.heap[index]->climax) {
    job_t *tmp = scheduler.heap[parent];
    scheduler.heap[parent] = scheduler.heap[index];
    scheduler.heap[index] = tmp;
    scheduler.heap[parent]->position = parent;
    scheduler.heap[index]->position = index;
    index = parent;
    parent = (index - 1) / 2;
  }
  if (index == 0) {
    canzero_request_update(job->climax);
  }
}

static void scheduler_schedule(job_t *job) {
  if (scheduler.size >= SCHEDULER_HEAP_SIZE) {
    return;
  }
  job->position = scheduler.size;
  scheduler.heap[scheduler.size] = job;
  scheduler.size += 1;
  scheduler_promote_job(job);
}

static int scheduler_continue(job_t **job, uint32_t time) {
  *job = scheduler.heap[0];
  return scheduler.heap[0]->climax <= time;
}

static void scheduler_reschedule(uint32_t climax) {
  job_t *job = scheduler.heap[0];
  job->climax = climax;
  int index = 0;
  int hsize = scheduler.size / 2;
  while (index < hsize) {
    int left = index * 2 + 1;
    int right = left + 1;
    int min;
    if (right < scheduler.size &&
        scheduler.heap[left]->climax >= scheduler.heap[right]->climax) {
      min = right;
    } else {
    min = left;
    }
    if (climax <= scheduler.heap[min]->climax) {
      break;
    }
    scheduler.heap[index] = scheduler.heap[min];
    scheduler.heap[index]->position = index;
    index = min;
  }
  scheduler.heap[index] = job;
  scheduler.heap[index]->position = index;
}
static void scheduler_unschedule() {
  scheduler.heap[0] = scheduler.heap[scheduler.size - 1];
  scheduler.heap[0]->position = 0;
  scheduler.size -= 1;
  scheduler_reschedule(scheduler.heap[0]->climax);
}
static const uint32_t get_resp_fragmentation_interval = 10;
static void schedule_get_resp_fragmentation_job(uint32_t *fragmentation_buffer, uint8_t size, uint8_t od_index, uint8_t client_id) {
  job_t *fragmentation_job = job_pool_allocator_alloc();
  fragmentation_job->climax = canzero_get_time() + get_resp_fragmentation_interval;
  fragmentation_job->tag = GET_RESP_FRAGMENTATION_JOB_TAG;
  fragmentation_job->job.get_fragmentation_job.buffer = fragmentation_buffer;
  fragmentation_job->job.get_fragmentation_job.offset = 1;
  fragmentation_job->job.get_fragmentation_job.size = size;
  fragmentation_job->job.get_fragmentation_job.od_index = od_index;
  fragmentation_job->job.get_fragmentation_job.client_id = client_id;
  scheduler_schedule(fragmentation_job);
}

static job_t heartbeat_job;
static const uint32_t heartbeat_interval = 100;
static void schedule_heartbeat_job() {
  heartbeat_job.climax = canzero_get_time();
  heartbeat_job.tag = HEARTBEAT_JOB_TAG;
  scheduler_schedule(&heartbeat_job);
}

static job_t heartbeat_wdg_job;
static const uint32_t heartbeat_wdg_tick_duration = 50;
unsigned int wdg_job_can0_static_wdg_armed[node_id_count];
int wdg_job_can0_static_tick_countdowns[node_id_count];
unsigned int wdg_job_can0_dynamic_wdg_armed[MAX_DYN_HEARTBEATS];
int wdg_job_can0_dynamic_tick_countdowns[MAX_DYN_HEARTBEATS];
unsigned int wdg_job_can1_static_wdg_armed[node_id_count];
int wdg_job_can1_static_tick_countdowns[node_id_count];
unsigned int wdg_job_can1_dynamic_wdg_armed[MAX_DYN_HEARTBEATS];
int wdg_job_can1_dynamic_tick_countdowns[MAX_DYN_HEARTBEATS];

static void schedule_heartbeat_wdg_job() {
  heartbeat_wdg_job.climax = canzero_get_time() + 100;
  heartbeat_wdg_job.tag = HEARTBEAT_WDG_JOB_TAG;
  heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed = wdg_job_can0_static_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns = wdg_job_can0_static_tick_countdowns;
  heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed = wdg_job_can0_dynamic_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns = wdg_job_can0_dynamic_tick_countdowns;
  heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed = wdg_job_can1_static_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns = wdg_job_can1_static_tick_countdowns;
  heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed = wdg_job_can1_dynamic_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns = wdg_job_can1_dynamic_tick_countdowns;
  for (unsigned int i = 0; i < node_id_count; ++i) {
    heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[i] = 10;
    heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[i] = 0;
    heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[i] = 10;
    heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[i] = 0;
  }
  for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
    heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[i] = 4;
    heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[i] = 0;
    heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[i] = 4;
    heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[i] = 0;
  }
  scheduler_schedule(&heartbeat_wdg_job);
}

static job_t state_interval_job;
static const uint32_t state_interval = 1;
static void schedule_state_interval_job(){
  uint32_t time = canzero_get_time();
  state_interval_job.climax = time + state_interval;
  state_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  state_interval_job.job.stream_job.stream_id = 0;
  state_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&state_interval_job);
}
static job_t errors_interval_job;
static const uint32_t errors_interval = 1;
static void schedule_errors_interval_job(){
  uint32_t time = canzero_get_time();
  errors_interval_job.climax = time + errors_interval;
  errors_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  errors_interval_job.job.stream_job.stream_id = 1;
  errors_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&errors_interval_job);
}
static job_t debug_interval_job;
static const uint32_t debug_interval = 500;
static void schedule_debug_interval_job(){
  uint32_t time = canzero_get_time();
  debug_interval_job.climax = time + debug_interval;
  debug_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  debug_interval_job.job.stream_job.stream_id = 2;
  debug_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&debug_interval_job);
}
static job_t voltage_and_currents_interval_job;
static const uint32_t voltage_and_currents_interval = 5;
static void schedule_voltage_and_currents_interval_job(){
  uint32_t time = canzero_get_time();
  voltage_and_currents_interval_job.climax = time + voltage_and_currents_interval;
  voltage_and_currents_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  voltage_and_currents_interval_job.job.stream_job.stream_id = 3;
  voltage_and_currents_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&voltage_and_currents_interval_job);
}
static job_t airgaps_interval_job;
static const uint32_t airgaps_interval = 100;
static void schedule_airgaps_interval_job(){
  uint32_t time = canzero_get_time();
  airgaps_interval_job.climax = time + airgaps_interval;
  airgaps_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  airgaps_interval_job.job.stream_job.stream_id = 4;
  airgaps_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&airgaps_interval_job);
}

static void schedule_jobs(uint32_t time) {
  for (uint8_t i = 0; i < 100; ++i) {
    canzero_enter_critical();
    job_t *job;
    if (!scheduler_continue(&job, time)) {
      canzero_exit_critical();
      return;
    }
    switch (job->tag) {
      case STREAM_INTERVAL_JOB_TAG: {
        switch (job->job.stream_job.stream_id) {
      case 0: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_guidance_board_front_stream_state stream_message;
        stream_message.m_state = __oe_state;
        stream_message.m_sdc_status = __oe_sdc_status;
        stream_message.m_command = __oe_command;
        stream_message.m_control_active = __oe_control_active;
        stream_message.m_precharge_status = __oe_precharge_status;
        stream_message.m_feedthrough_status = __oe_feedthrough_status;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_guidance_board_front_stream_state(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 1: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 1000);
        canzero_exit_critical();
        canzero_message_guidance_board_front_stream_errors stream_message;
        stream_message.m_error_level_vdc_voltage = __oe_error_level_vdc_voltage;
        stream_message.m_error_arming_failed = __oe_error_arming_failed;
        stream_message.m_error_precharge_failed = __oe_error_precharge_failed;
        stream_message.m_error_level_magnet_current_left = __oe_error_level_magnet_current_left;
        stream_message.m_error_level_magnet_current_right = __oe_error_level_magnet_current_right;
        stream_message.m_error_level_input_current = __oe_error_level_input_current;
        stream_message.m_error_level_magnet_temperature_left = __oe_error_level_magnet_temperature_left;
        stream_message.m_error_level_magnet_temperature_right = __oe_error_level_magnet_temperature_right;
        stream_message.m_error_level_mcu_temperature = __oe_error_level_mcu_temperature;
        stream_message.m_error_heartbeat_miss = __oe_error_heartbeat_miss;
        stream_message.m_assertion_fault = __oe_assertion_fault;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_guidance_board_front_stream_errors(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 2: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_guidance_board_front_stream_debug stream_message;
        stream_message.m_loop_frequency = __oe_loop_frequency;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_guidance_board_front_stream_debug(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 3: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 5);
        canzero_exit_critical();
        canzero_message_guidance_board_front_stream_voltage_and_currents stream_message;
        stream_message.m_vdc_voltage = __oe_vdc_voltage;
        stream_message.m_current_left = __oe_current_left;
        stream_message.m_current_right = __oe_current_right;
        stream_message.m_input_current = __oe_input_current;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_guidance_board_front_stream_voltage_and_currents(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 4: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 100);
        canzero_exit_critical();
        canzero_message_guidance_board_front_stream_airgaps stream_message;
        stream_message.m_outer_airgap_left = __oe_outer_airgap_left;
        stream_message.m_inner_airgap_left = __oe_inner_airgap_left;
        stream_message.m_outer_airgap_right = __oe_outer_airgap_right;
        stream_message.m_inner_airgap_right = __oe_inner_airgap_right;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_guidance_board_front_stream_airgaps(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
        default:
          canzero_exit_critical();
          break;
        }
        break;
      }
      case HEARTBEAT_JOB_TAG: {
        scheduler_reschedule(time + heartbeat_interval);
        canzero_exit_critical();
        canzero_frame heartbeat_frame;
        canzero_message_heartbeat_can0 heartbeat_can0;
        heartbeat_can0.m_node_id = node_id_guidance_board_front;
        heartbeat_can0.m_unregister = 0;
        heartbeat_can0.m_ticks_next = 4;
        canzero_serialize_canzero_message_heartbeat_can0(&heartbeat_can0, &heartbeat_frame);
        canzero_can0_send(&heartbeat_frame);
        canzero_message_heartbeat_can1 heartbeat_can1;
        heartbeat_can1.m_node_id = node_id_guidance_board_front;
        heartbeat_can1.m_unregister = 0;
        heartbeat_can1.m_ticks_next = 4;
        canzero_serialize_canzero_message_heartbeat_can1(&heartbeat_can1, &heartbeat_frame);
        canzero_can1_send(&heartbeat_frame);
        break;
      }
      case HEARTBEAT_WDG_JOB_TAG: {
        scheduler_reschedule(time + heartbeat_wdg_tick_duration);
        canzero_exit_critical();
        for (unsigned int i = 0; i < node_id_count; ++i) {
          heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[i];
          heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[i];
        }
        for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
          heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[i];
          heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[i];
        }
        for (unsigned int i = 0; i < node_id_count; ++i) {
          if (heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[i] <= 0) {
            canzero_can0_wdg_timeout(i);
          }
          if (heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[i] <= 0) {
            canzero_can1_wdg_timeout(i);
          }
        }
        for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
          if (heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[i] <= 0) {
            canzero_can0_wdg_timeout(node_id_count + i);
          }
          if (heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[i] <= 0) {
            canzero_can1_wdg_timeout(node_id_count + i);
          }
        }
        break;
      }
      case GET_RESP_FRAGMENTATION_JOB_TAG: {
        get_resp_fragmentation_job *fragmentation_job = &job->job.get_fragmentation_job;
        canzero_message_get_resp fragmentation_response;
        fragmentation_response.m_header.m_sof = 0;
        fragmentation_response.m_header.m_toggle = fragmentation_job->offset % 2;
        fragmentation_response.m_header.m_od_index = fragmentation_job->od_index;
        fragmentation_response.m_header.m_client_id = fragmentation_job->client_id;
        fragmentation_response.m_header.m_server_id = 0x3;
        fragmentation_response.m_data = fragmentation_job->buffer[fragmentation_job->offset];
        fragmentation_job->offset += 1;
        if (fragmentation_job->offset == fragmentation_job->size) {
          fragmentation_response.m_header.m_eof = 1;
          scheduler_unschedule();
        } else {
          fragmentation_response.m_header.m_eof = 0;
          scheduler_reschedule(time + get_resp_fragmentation_interval);
        }
        canzero_exit_critical();
        canzero_frame fragmentation_frame;
        canzero_serialize_canzero_message_get_resp(&fragmentation_response, &fragmentation_frame);
        canzero_can0_send(&fragmentation_frame);
        break;
      }
      default: {
        canzero_exit_critical();
        break;
      }
    }
  }
}

static uint32_t scheduler_next_job_timeout() {
  return scheduler.heap[0]->climax;
}

static uint32_t DMAMEM __oe_config_hash_rx_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_build_time_rx_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_magnet_current_rx_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_input_current_rx_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[7];
static PROGMEM void canzero_handle_get_req(canzero_frame* frame) {
  canzero_message_get_req msg;
  canzero_deserialize_canzero_message_get_req(frame, &msg);
  if (msg.m_header.m_server_id != node_id_guidance_board_front) {
    return;
  }
  canzero_message_get_resp resp{};
  switch (msg.m_header.m_od_index) {
  case 0: {
    {
      uint64_t masked = (__oe_config_hash & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
      __oe_config_hash_rx_fragmentation_buffer[0] = ((uint32_t*)&masked)[0];
      __oe_config_hash_rx_fragmentation_buffer[1] = ((uint32_t*)&masked)[1];
    }
    resp.m_data = __oe_config_hash_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_config_hash_rx_fragmentation_buffer, 2, 0, msg.m_header.m_client_id);
    break;
  }
  case 1: {
    __oe_build_time_rx_fragmentation_buffer[0] = (__oe_build_time.m_year & (0xFFFFFFFF >> (32 - 16)));
    __oe_build_time_rx_fragmentation_buffer[0] |= ((__oe_build_time.m_month & (0xFFFFFFFF >> (32 - 8))) << 16);
    __oe_build_time_rx_fragmentation_buffer[0] |= ((__oe_build_time.m_day & (0xFFFFFFFF >> (32 - 8))) << 24);
    __oe_build_time_rx_fragmentation_buffer[1] = (__oe_build_time.m_hour & (0xFFFFFFFF >> (32 - 8)));
    __oe_build_time_rx_fragmentation_buffer[1] |= ((__oe_build_time.m_min & (0xFFFFFFFF >> (32 - 8))) << 8);
    __oe_build_time_rx_fragmentation_buffer[1] |= ((__oe_build_time.m_sec & (0xFFFFFFFF >> (32 - 8))) << 16);

    resp.m_data = __oe_build_time_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_build_time_rx_fragmentation_buffer, 2, 1, msg.m_header.m_client_id);
    break;
  }
  case 2: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 3: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 4: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 5: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_precharge_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 6: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_feedthrough_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 7: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_control_active) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 8: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_arming_failed) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 9: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_precharge_failed) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 10: {
    resp.m_data |= min_u32((__oe_outer_airgap_left - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 11: {
    resp.m_data |= min_u32((__oe_inner_airgap_left - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 12: {
    resp.m_data |= min_u32((__oe_outer_airgap_right - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 13: {
    resp.m_data |= min_u32((__oe_inner_airgap_right - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 14: {
    resp.m_data |= min_u32((__oe_vdc_voltage - (0)) / 0.0015259021896696422, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 15: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_vdc_voltage) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 16: {
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_vdc_voltage.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_vdc_voltage.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_vdc_voltage.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_vdc_voltage.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_vdc_voltage.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_vdc_voltage.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[6] = (__oe_error_level_config_vdc_voltage.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_vdc_voltage.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
    __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_vdc_voltage.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

    resp.m_data = __oe_error_level_config_vdc_voltage_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_error_level_config_vdc_voltage_rx_fragmentation_buffer, 7, 16, msg.m_header.m_client_id);
    break;
  }
  case 17: {
    resp.m_data |= min_u32((__oe_current_left - (-25)) / 0.0007629510948348211, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 18: {
    resp.m_data |= min_u32((__oe_current_right - (-25)) / 0.0007629510948348211, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 19: {
    resp.m_data |= min_u32((__oe_input_current - (-25)) / 0.0007629510948348211, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 20: {
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_magnet_current.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_magnet_current.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_magnet_current.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_magnet_current.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_magnet_current.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_magnet_current.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[6] = (__oe_error_level_config_magnet_current.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_current.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
    __oe_error_level_config_magnet_current_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_current.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

    resp.m_data = __oe_error_level_config_magnet_current_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_error_level_config_magnet_current_rx_fragmentation_buffer, 7, 20, msg.m_header.m_client_id);
    break;
  }
  case 21: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_current_left) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 22: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_current_right) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 23: {
    __oe_error_level_config_input_current_rx_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_input_current.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_input_current.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_input_current.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_input_current.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_input_current.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_input_current.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[6] = (__oe_error_level_config_input_current.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
    __oe_error_level_config_input_current_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_input_current.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
    __oe_error_level_config_input_current_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_input_current.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

    resp.m_data = __oe_error_level_config_input_current_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_error_level_config_input_current_rx_fragmentation_buffer, 7, 23, msg.m_header.m_client_id);
    break;
  }
  case 24: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_input_current) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 25: {
    resp.m_data |= min_u32((__oe_magnet_temperature_left1 - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 26: {
    resp.m_data |= min_u32((__oe_magnet_temperature_left2 - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 27: {
    resp.m_data |= min_u32((__oe_magnet_temperature_left_max - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 28: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_temperature_left) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 29: {
    resp.m_data |= min_u32((__oe_magnet_temperature_right1 - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 30: {
    resp.m_data |= min_u32((__oe_magnet_temperature_right2 - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 31: {
    resp.m_data |= min_u32((__oe_magnet_temperature_right_max - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 32: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_temperature_right) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 33: {
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_magnet_temperature.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_magnet_temperature.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_magnet_temperature.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_magnet_temperature.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_magnet_temperature.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_magnet_temperature.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[6] = (__oe_error_level_config_magnet_temperature.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_temperature.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
    __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_temperature.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

    resp.m_data = __oe_error_level_config_magnet_temperature_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_error_level_config_magnet_temperature_rx_fragmentation_buffer, 7, 33, msg.m_header.m_client_id);
    break;
  }
  case 34: {
    resp.m_data |= min_u32((__oe_mcu_temperature - (-1)) / 0.592156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 35: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_mcu_temperature) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 36: {
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_mcu_temperature.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_mcu_temperature.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_mcu_temperature.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_mcu_temperature.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_mcu_temperature.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_mcu_temperature.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[6] = (__oe_error_level_config_mcu_temperature.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_mcu_temperature.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
    __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[6] |= ((__oe_error_level_config_mcu_temperature.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

    resp.m_data = __oe_error_level_config_mcu_temperature_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_error_level_config_mcu_temperature_rx_fragmentation_buffer, 7, 36, msg.m_header.m_client_id);
    break;
  }
  case 37: {
    resp.m_data |= min_u32((__oe_loop_frequency - (0)) / 0.00000023283064370807974, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 38: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_assertion_fault) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 39: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_heartbeat_miss) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 40: {
    resp.m_data |= min_u32((__oe_gamepad_lt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 41: {
    resp.m_data |= min_u32((__oe_gamepad_rt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 42: {
    resp.m_data |= min_u32((__oe_gamepad_lsb_x - (-1)) / 0.00784313725490196, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 43: {
    resp.m_data |= min_u32((__oe_gamepad_lsb_y - (-1)) / 0.00784313725490196, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 44: {
    resp.m_data |= min_u32((__oe_gamepad_rsb_x - (-1)) / 0.00784313725490196, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 45: {
    resp.m_data |= min_u32((__oe_gamepad_rsb_y - (-1)) / 0.00784313725490196, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 46: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_gamepad_lt1_down) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 47: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_gamepad_rt1_down) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 48: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_gamepad_x_down) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  }
  resp.m_header.m_od_index = msg.m_header.m_od_index;
  resp.m_header.m_client_id = msg.m_header.m_client_id;
  resp.m_header.m_server_id = msg.m_header.m_server_id;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_get_resp(&resp, &resp_frame);
  canzero_can0_send(&resp_frame);
}
static uint32_t DMAMEM config_hash_tmp_tx_fragmentation_buffer[2];
static uint32_t DMAMEM config_hash_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM build_time_tmp_tx_fragmentation_buffer[2];
static uint32_t DMAMEM build_time_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[7];
static uint32_t DMAMEM error_level_config_vdc_voltage_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM error_level_config_magnet_current_tmp_tx_fragmentation_buffer[7];
static uint32_t DMAMEM error_level_config_magnet_current_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM error_level_config_input_current_tmp_tx_fragmentation_buffer[7];
static uint32_t DMAMEM error_level_config_input_current_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[7];
static uint32_t DMAMEM error_level_config_magnet_temperature_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[7];
static uint32_t DMAMEM error_level_config_mcu_temperature_tmp_tx_fragmentation_offset = 0;
static PROGMEM void canzero_handle_set_req(canzero_frame* frame) {
  canzero_message_set_req msg;
  canzero_deserialize_canzero_message_set_req(frame, &msg);
  if (msg.m_header.m_server_id != 3) {
    return;
  }
  canzero_message_set_resp resp{};
  switch (msg.m_header.m_od_index) {
  case 0 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      config_hash_tmp_tx_fragmentation_offset = 0;
    }else {
      config_hash_tmp_tx_fragmentation_offset += 1;
      if (config_hash_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    config_hash_tmp_tx_fragmentation_buffer[config_hash_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    uint64_t config_hash_tmp;
    config_hash_tmp = (uint64_t)config_hash_tmp_tx_fragmentation_buffer[0] | (((uint64_t)(config_hash_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) << 32);
    canzero_set_config_hash(config_hash_tmp);
    break;
  }
  case 1 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      build_time_tmp_tx_fragmentation_offset = 0;
    }else {
      build_time_tmp_tx_fragmentation_offset += 1;
      if (build_time_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    build_time_tmp_tx_fragmentation_buffer[build_time_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    date_time build_time_tmp;
    build_time_tmp.m_year = (build_time_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 16)));
    build_time_tmp.m_month = (build_time_tmp_tx_fragmentation_buffer[0] >> 16) & (0xFFFFFFFF >> (32 - 8));
    build_time_tmp.m_day = (build_time_tmp_tx_fragmentation_buffer[0] >> 24) & (0xFFFFFFFF >> (32 - 8));
    build_time_tmp.m_hour = (build_time_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 8)));
    build_time_tmp.m_min = (build_time_tmp_tx_fragmentation_buffer[1] >> 8) & (0xFFFFFFFF >> (32 - 8));
    build_time_tmp.m_sec = (build_time_tmp_tx_fragmentation_buffer[1] >> 16) & (0xFFFFFFFF >> (32 - 8));
    canzero_set_build_time(build_time_tmp);
    break;
  }
  case 2 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_state state_tmp;
    state_tmp = ((guidance_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_state(state_tmp);
    break;
  }
  case 3 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status sdc_status_tmp;
    sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_sdc_status(sdc_status_tmp);
    break;
  }
  case 4 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_command command_tmp;
    command_tmp = ((guidance_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_command(command_tmp);
    break;
  }
  case 5 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status precharge_status_tmp;
    precharge_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_precharge_status(precharge_status_tmp);
    break;
  }
  case 6 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status feedthrough_status_tmp;
    feedthrough_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_feedthrough_status(feedthrough_status_tmp);
    break;
  }
  case 7 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    bool_t control_active_tmp;
    control_active_tmp = ((bool_t)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_control_active(control_active_tmp);
    break;
  }
  case 8 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_flag error_arming_failed_tmp;
    error_arming_failed_tmp = ((error_flag)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_arming_failed(error_arming_failed_tmp);
    break;
  }
  case 9 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_flag error_precharge_failed_tmp;
    error_precharge_failed_tmp = ((error_flag)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_precharge_failed(error_precharge_failed_tmp);
    break;
  }
  case 10 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float outer_airgap_left_tmp;
    outer_airgap_left_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0);
    canzero_set_outer_airgap_left(outer_airgap_left_tmp);
    break;
  }
  case 11 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float inner_airgap_left_tmp;
    inner_airgap_left_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0);
    canzero_set_inner_airgap_left(inner_airgap_left_tmp);
    break;
  }
  case 12 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float outer_airgap_right_tmp;
    outer_airgap_right_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0);
    canzero_set_outer_airgap_right(outer_airgap_right_tmp);
    break;
  }
  case 13 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float inner_airgap_right_tmp;
    inner_airgap_right_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0);
    canzero_set_inner_airgap_right(inner_airgap_right_tmp);
    break;
  }
  case 14 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float vdc_voltage_tmp;
    vdc_voltage_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + 0);
    canzero_set_vdc_voltage(vdc_voltage_tmp);
    break;
  }
  case 15 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_vdc_voltage_tmp;
    error_level_vdc_voltage_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_vdc_voltage(error_level_vdc_voltage_tmp);
    break;
  }
  case 16 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      error_level_config_vdc_voltage_tmp_tx_fragmentation_offset = 0;
    }else {
      error_level_config_vdc_voltage_tmp_tx_fragmentation_offset += 1;
      if (error_level_config_vdc_voltage_tmp_tx_fragmentation_offset >= 7) {
        return;
      }
    }
    error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[error_level_config_vdc_voltage_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    error_level_config error_level_config_vdc_voltage_tmp;
    error_level_config_vdc_voltage_tmp.m_info_thresh = ((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_vdc_voltage_tmp.m_info_timeout = ((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_vdc_voltage_tmp.m_warning_thresh = ((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[2] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_vdc_voltage_tmp.m_warning_timeout = ((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[3] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_vdc_voltage_tmp.m_error_thresh = ((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[4] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_vdc_voltage_tmp.m_error_timeout = ((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[5] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_vdc_voltage_tmp.m_ignore_info = ((bool_t)((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[6] & (0xFFFFFFFF >> (32 - 1)))));
    error_level_config_vdc_voltage_tmp.m_ignore_warning = ((bool_t)((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[6] >> 1) & (0xFFFFFFFF >> (32 - 1))));
    error_level_config_vdc_voltage_tmp.m_ignore_error = ((bool_t)((error_level_config_vdc_voltage_tmp_tx_fragmentation_buffer[6] >> 2) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_level_config_vdc_voltage(error_level_config_vdc_voltage_tmp);
    break;
  }
  case 17 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float current_left_tmp;
    current_left_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0007629510948348211 + -25);
    canzero_set_current_left(current_left_tmp);
    break;
  }
  case 18 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float current_right_tmp;
    current_right_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0007629510948348211 + -25);
    canzero_set_current_right(current_right_tmp);
    break;
  }
  case 19 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float input_current_tmp;
    input_current_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0007629510948348211 + -25);
    canzero_set_input_current(input_current_tmp);
    break;
  }
  case 20 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      error_level_config_magnet_current_tmp_tx_fragmentation_offset = 0;
    }else {
      error_level_config_magnet_current_tmp_tx_fragmentation_offset += 1;
      if (error_level_config_magnet_current_tmp_tx_fragmentation_offset >= 7) {
        return;
      }
    }
    error_level_config_magnet_current_tmp_tx_fragmentation_buffer[error_level_config_magnet_current_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    error_level_config error_level_config_magnet_current_tmp;
    error_level_config_magnet_current_tmp.m_info_thresh = ((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_magnet_current_tmp.m_info_timeout = ((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_magnet_current_tmp.m_warning_thresh = ((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[2] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_magnet_current_tmp.m_warning_timeout = ((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[3] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_magnet_current_tmp.m_error_thresh = ((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[4] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_magnet_current_tmp.m_error_timeout = ((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[5] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_magnet_current_tmp.m_ignore_info = ((bool_t)((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[6] & (0xFFFFFFFF >> (32 - 1)))));
    error_level_config_magnet_current_tmp.m_ignore_warning = ((bool_t)((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[6] >> 1) & (0xFFFFFFFF >> (32 - 1))));
    error_level_config_magnet_current_tmp.m_ignore_error = ((bool_t)((error_level_config_magnet_current_tmp_tx_fragmentation_buffer[6] >> 2) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_level_config_magnet_current(error_level_config_magnet_current_tmp);
    break;
  }
  case 21 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_magnet_current_left_tmp;
    error_level_magnet_current_left_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_magnet_current_left(error_level_magnet_current_left_tmp);
    break;
  }
  case 22 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_magnet_current_right_tmp;
    error_level_magnet_current_right_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_magnet_current_right(error_level_magnet_current_right_tmp);
    break;
  }
  case 23 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      error_level_config_input_current_tmp_tx_fragmentation_offset = 0;
    }else {
      error_level_config_input_current_tmp_tx_fragmentation_offset += 1;
      if (error_level_config_input_current_tmp_tx_fragmentation_offset >= 7) {
        return;
      }
    }
    error_level_config_input_current_tmp_tx_fragmentation_buffer[error_level_config_input_current_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    error_level_config error_level_config_input_current_tmp;
    error_level_config_input_current_tmp.m_info_thresh = ((error_level_config_input_current_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_input_current_tmp.m_info_timeout = ((error_level_config_input_current_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_input_current_tmp.m_warning_thresh = ((error_level_config_input_current_tmp_tx_fragmentation_buffer[2] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_input_current_tmp.m_warning_timeout = ((error_level_config_input_current_tmp_tx_fragmentation_buffer[3] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_input_current_tmp.m_error_thresh = ((error_level_config_input_current_tmp_tx_fragmentation_buffer[4] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_input_current_tmp.m_error_timeout = ((error_level_config_input_current_tmp_tx_fragmentation_buffer[5] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_input_current_tmp.m_ignore_info = ((bool_t)((error_level_config_input_current_tmp_tx_fragmentation_buffer[6] & (0xFFFFFFFF >> (32 - 1)))));
    error_level_config_input_current_tmp.m_ignore_warning = ((bool_t)((error_level_config_input_current_tmp_tx_fragmentation_buffer[6] >> 1) & (0xFFFFFFFF >> (32 - 1))));
    error_level_config_input_current_tmp.m_ignore_error = ((bool_t)((error_level_config_input_current_tmp_tx_fragmentation_buffer[6] >> 2) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_level_config_input_current(error_level_config_input_current_tmp);
    break;
  }
  case 24 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_input_current_tmp;
    error_level_input_current_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_input_current(error_level_input_current_tmp);
    break;
  }
  case 25 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float magnet_temperature_left1_tmp;
    magnet_temperature_left1_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_left1(magnet_temperature_left1_tmp);
    break;
  }
  case 26 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float magnet_temperature_left2_tmp;
    magnet_temperature_left2_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_left2(magnet_temperature_left2_tmp);
    break;
  }
  case 27 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float magnet_temperature_left_max_tmp;
    magnet_temperature_left_max_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_left_max(magnet_temperature_left_max_tmp);
    break;
  }
  case 28 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_magnet_temperature_left_tmp;
    error_level_magnet_temperature_left_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_magnet_temperature_left(error_level_magnet_temperature_left_tmp);
    break;
  }
  case 29 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float magnet_temperature_right1_tmp;
    magnet_temperature_right1_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_right1(magnet_temperature_right1_tmp);
    break;
  }
  case 30 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float magnet_temperature_right2_tmp;
    magnet_temperature_right2_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_right2(magnet_temperature_right2_tmp);
    break;
  }
  case 31 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float magnet_temperature_right_max_tmp;
    magnet_temperature_right_max_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_right_max(magnet_temperature_right_max_tmp);
    break;
  }
  case 32 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_magnet_temperature_right_tmp;
    error_level_magnet_temperature_right_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_magnet_temperature_right(error_level_magnet_temperature_right_tmp);
    break;
  }
  case 33 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      error_level_config_magnet_temperature_tmp_tx_fragmentation_offset = 0;
    }else {
      error_level_config_magnet_temperature_tmp_tx_fragmentation_offset += 1;
      if (error_level_config_magnet_temperature_tmp_tx_fragmentation_offset >= 7) {
        return;
      }
    }
    error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[error_level_config_magnet_temperature_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    error_level_config error_level_config_magnet_temperature_tmp;
    error_level_config_magnet_temperature_tmp.m_info_thresh = ((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_magnet_temperature_tmp.m_info_timeout = ((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_magnet_temperature_tmp.m_warning_thresh = ((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[2] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_magnet_temperature_tmp.m_warning_timeout = ((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[3] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_magnet_temperature_tmp.m_error_thresh = ((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[4] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_magnet_temperature_tmp.m_error_timeout = ((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[5] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_magnet_temperature_tmp.m_ignore_info = ((bool_t)((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[6] & (0xFFFFFFFF >> (32 - 1)))));
    error_level_config_magnet_temperature_tmp.m_ignore_warning = ((bool_t)((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[6] >> 1) & (0xFFFFFFFF >> (32 - 1))));
    error_level_config_magnet_temperature_tmp.m_ignore_error = ((bool_t)((error_level_config_magnet_temperature_tmp_tx_fragmentation_buffer[6] >> 2) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_level_config_magnet_temperature(error_level_config_magnet_temperature_tmp);
    break;
  }
  case 34 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float mcu_temperature_tmp;
    mcu_temperature_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_mcu_temperature(mcu_temperature_tmp);
    break;
  }
  case 35 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_level error_level_mcu_temperature_tmp;
    error_level_mcu_temperature_tmp = ((error_level)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_error_level_mcu_temperature(error_level_mcu_temperature_tmp);
    break;
  }
  case 36 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      error_level_config_mcu_temperature_tmp_tx_fragmentation_offset = 0;
    }else {
      error_level_config_mcu_temperature_tmp_tx_fragmentation_offset += 1;
      if (error_level_config_mcu_temperature_tmp_tx_fragmentation_offset >= 7) {
        return;
      }
    }
    error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[error_level_config_mcu_temperature_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    error_level_config error_level_config_mcu_temperature_tmp;
    error_level_config_mcu_temperature_tmp.m_info_thresh = ((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_mcu_temperature_tmp.m_info_timeout = ((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_mcu_temperature_tmp.m_warning_thresh = ((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[2] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_mcu_temperature_tmp.m_warning_timeout = ((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[3] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_mcu_temperature_tmp.m_error_thresh = ((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[4] & (0xFFFFFFFF >> (32 - 32)))) * 0.0000004656612874161595 + -1000;
    error_level_config_mcu_temperature_tmp.m_error_timeout = ((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[5] & (0xFFFFFFFF >> (32 - 32)))) * 0.000000013969838622484784 + 0;
    error_level_config_mcu_temperature_tmp.m_ignore_info = ((bool_t)((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[6] & (0xFFFFFFFF >> (32 - 1)))));
    error_level_config_mcu_temperature_tmp.m_ignore_warning = ((bool_t)((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[6] >> 1) & (0xFFFFFFFF >> (32 - 1))));
    error_level_config_mcu_temperature_tmp.m_ignore_error = ((bool_t)((error_level_config_mcu_temperature_tmp_tx_fragmentation_buffer[6] >> 2) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_level_config_mcu_temperature(error_level_config_mcu_temperature_tmp);
    break;
  }
  case 37 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float loop_frequency_tmp;
    loop_frequency_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.00000023283064370807974 + 0);
    canzero_set_loop_frequency(loop_frequency_tmp);
    break;
  }
  case 38 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_flag assertion_fault_tmp;
    assertion_fault_tmp = ((error_flag)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_assertion_fault(assertion_fault_tmp);
    break;
  }
  case 39 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_flag error_heartbeat_miss_tmp;
    error_heartbeat_miss_tmp = ((error_flag)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_heartbeat_miss(error_heartbeat_miss_tmp);
    break;
  }
  case 40 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_lt2_tmp;
    gamepad_lt2_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0);
    canzero_set_gamepad_lt2(gamepad_lt2_tmp);
    break;
  }
  case 41 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_rt2_tmp;
    gamepad_rt2_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0);
    canzero_set_gamepad_rt2(gamepad_rt2_tmp);
    break;
  }
  case 42 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_lsb_x_tmp;
    gamepad_lsb_x_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1);
    canzero_set_gamepad_lsb_x(gamepad_lsb_x_tmp);
    break;
  }
  case 43 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_lsb_y_tmp;
    gamepad_lsb_y_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1);
    canzero_set_gamepad_lsb_y(gamepad_lsb_y_tmp);
    break;
  }
  case 44 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_rsb_x_tmp;
    gamepad_rsb_x_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1);
    canzero_set_gamepad_rsb_x(gamepad_rsb_x_tmp);
    break;
  }
  case 45 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_rsb_y_tmp;
    gamepad_rsb_y_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1);
    canzero_set_gamepad_rsb_y(gamepad_rsb_y_tmp);
    break;
  }
  case 46 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    bool_t gamepad_lt1_down_tmp;
    gamepad_lt1_down_tmp = ((bool_t)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_gamepad_lt1_down(gamepad_lt1_down_tmp);
    break;
  }
  case 47 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    bool_t gamepad_rt1_down_tmp;
    gamepad_rt1_down_tmp = ((bool_t)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_gamepad_rt1_down(gamepad_rt1_down_tmp);
    break;
  }
  case 48 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    bool_t gamepad_x_down_tmp;
    gamepad_x_down_tmp = ((bool_t)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_gamepad_x_down(gamepad_x_down_tmp);
    break;
  }
  default:
    return;
  }
  resp.m_header.m_od_index = msg.m_header.m_od_index;
  resp.m_header.m_client_id = msg.m_header.m_client_id;
  resp.m_header.m_server_id = msg.m_header.m_server_id;
  resp.m_header.m_erno = set_resp_erno_Success;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_set_resp(&resp, &resp_frame);
  canzero_can1_send(&resp_frame);

}
static void canzero_handle_gamepad_stream_input(canzero_frame* frame) {
  canzero_message_gamepad_stream_input msg;
  canzero_deserialize_canzero_message_gamepad_stream_input(frame, &msg);
  canzero_set_gamepad_lt2(msg.m_lt2);
  canzero_set_gamepad_rt2(msg.m_rt2);
  canzero_set_gamepad_lsb_x(msg.m_lsb_x);
  canzero_set_gamepad_lsb_y(msg.m_lsb_y);
  canzero_set_gamepad_rsb_x(msg.m_rsb_x);
  canzero_set_gamepad_rsb_y(msg.m_rsb_y);
  canzero_set_gamepad_lt1_down(msg.m_lt1_down);
  canzero_set_gamepad_rt1_down(msg.m_rt1_down);
  canzero_set_gamepad_x_down(msg.m_x_down);
}
 void canzero_handle_heartbeat_can0(canzero_frame* frame) {
  canzero_message_heartbeat_can0 msg;
  canzero_deserialize_canzero_message_heartbeat_can0(frame, &msg);

  if (msg.m_node_id < node_id_count) {   // static heartbeat
    if (msg.m_unregister != 0) {  // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[msg.m_node_id] = 0;
    } else { // register registers for all buses
      heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[msg.m_node_id] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[msg.m_node_id] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[msg.m_node_id] <= 0 &&
        msg.m_ticks_next > 0) {
      canzero_can0_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[msg.m_node_id] = msg.m_ticks_next;
  } else {  // dynamic heartbeat
    if (msg.m_unregister != 0) { // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 0;
    } else { // register registers all buses
      heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[msg.m_node_id - node_id_count] <= 0 
        && msg.m_ticks_next > 0) {
      canzero_can0_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[msg.m_node_id - node_id_count]
      = msg.m_ticks_next;
  }
}
 void canzero_handle_heartbeat_can1(canzero_frame* frame) {
  canzero_message_heartbeat_can1 msg;
  canzero_deserialize_canzero_message_heartbeat_can1(frame, &msg);

  if (msg.m_node_id < node_id_count) {   // static heartbeat
    if (msg.m_unregister != 0) {  // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[msg.m_node_id] = 0;
    } else { // register registers for all buses
      heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[msg.m_node_id] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[msg.m_node_id] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[msg.m_node_id] <= 0 &&
        msg.m_ticks_next > 0) {
      canzero_can1_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[msg.m_node_id] = msg.m_ticks_next;
  } else {  // dynamic heartbeat
    if (msg.m_unregister != 0) { // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 0;
    } else { // register registers all buses
      heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[msg.m_node_id - node_id_count] <= 0 
        && msg.m_ticks_next > 0) {
      canzero_can1_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[msg.m_node_id - node_id_count]
      = msg.m_ticks_next;
  }
}
void canzero_can0_poll() {
  canzero_frame frame;
  while (canzero_can0_recv(&frame)) {
    switch (frame.id) {
      case 0x17E:
        canzero_handle_get_req(&frame);
        break;
      case 0x1E5:
        canzero_handle_heartbeat_can0(&frame);
        break;
    }
  }
}
void canzero_can1_poll() {
  canzero_frame frame;
  while (canzero_can1_recv(&frame)) {
    switch (frame.id) {
      case 0x1BE:
        canzero_handle_set_req(&frame);
        break;
      case 0xBF:
        canzero_handle_gamepad_stream_input(&frame);
        break;
      case 0x1E4:
        canzero_handle_heartbeat_can1(&frame);
        break;
    }
  }
}
uint32_t canzero_update_continue(uint32_t time){
  schedule_jobs(time);
  return scheduler_next_job_timeout();
}
#define COMPUTE_BUILD_YEAR \
    ( (__DATE__[ 7] - '0') * 1000 + \
        (__DATE__[ 8] - '0') *  100 + \
        (__DATE__[ 9] - '0') *   10 + \
        (__DATE__[10] - '0') \
    )
#define COMPUTE_BUILD_DAY \
    ( \
        ((__DATE__[4] >= '0') ? (__DATE__[4] - '0') * 10 : 0) + \
        (__DATE__[5] - '0') \
    )
#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')
#define COMPUTE_BUILD_MONTH \
    ( \
        (BUILD_MONTH_IS_JAN) ?  1 : \
        (BUILD_MONTH_IS_FEB) ?  2 : \
        (BUILD_MONTH_IS_MAR) ?  3 : \
        (BUILD_MONTH_IS_APR) ?  4 : \
        (BUILD_MONTH_IS_MAY) ?  5 : \
        (BUILD_MONTH_IS_JUN) ?  6 : \
        (BUILD_MONTH_IS_JUL) ?  7 : \
        (BUILD_MONTH_IS_AUG) ?  8 : \
        (BUILD_MONTH_IS_SEP) ?  9 : \
        (BUILD_MONTH_IS_OCT) ? 10 : \
        (BUILD_MONTH_IS_NOV) ? 11 : \
        (BUILD_MONTH_IS_DEC) ? 12 : \
        /* error default */  99 \
    )
#define COMPUTE_BUILD_HOUR ((__TIME__[0] - '0') * 10 + __TIME__[1] - '0')
#define COMPUTE_BUILD_MIN  ((__TIME__[3] - '0') * 10 + __TIME__[4] - '0')
#define COMPUTE_BUILD_SEC  ((__TIME__[6] - '0') * 10 + __TIME__[7] - '0')
#define BUILD_DATE_IS_BAD (__DATE__[0] == '?')
#define BUILD_YEAR  ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_YEAR)
#define BUILD_MONTH ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_MONTH)
#define BUILD_DAY   ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_DAY)
#define BUILD_TIME_IS_BAD (__TIME__[0] == '?')
#define BUILD_HOUR  ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_HOUR)
#define BUILD_MIN   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_MIN)
#define BUILD_SEC   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_SEC)
void canzero_init() {
  __oe_config_hash = 3616101509144958858ull;
  __oe_build_time = {
    .m_year = BUILD_YEAR,
    .m_month = BUILD_MONTH,
    .m_day = BUILD_DAY,
    .m_hour = BUILD_HOUR,
    .m_min = BUILD_MIN,
    .m_sec = BUILD_SEC
  };
  canzero_can0_setup(1000000, NULL, 0);
  canzero_can1_setup(1000000, NULL, 0);

  job_pool_allocator_init();
  scheduler.size = 0;
  schedule_heartbeat_job();
  schedule_heartbeat_wdg_job();
  schedule_state_interval_job();
  schedule_errors_interval_job();
  schedule_debug_interval_job();
  schedule_voltage_and_currents_interval_job();
  schedule_airgaps_interval_job();

}
void canzero_set_state(guidance_state value) {
  extern guidance_state __oe_state;
  if (__oe_state != value) {
    __oe_state = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_sdc_status(sdc_status value) {
  extern sdc_status __oe_sdc_status;
  if (__oe_sdc_status != value) {
    __oe_sdc_status = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_command(guidance_command value) {
  extern guidance_command __oe_command;
  if (__oe_command != value) {
    __oe_command = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_precharge_status(sdc_status value) {
  extern sdc_status __oe_precharge_status;
  if (__oe_precharge_status != value) {
    __oe_precharge_status = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_feedthrough_status(sdc_status value) {
  extern sdc_status __oe_feedthrough_status;
  if (__oe_feedthrough_status != value) {
    __oe_feedthrough_status = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_control_active(bool_t value) {
  extern bool_t __oe_control_active;
  if (__oe_control_active != value) {
    __oe_control_active = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_error_arming_failed(error_flag value) {
  extern error_flag __oe_error_arming_failed;
  if (__oe_error_arming_failed != value) {
    __oe_error_arming_failed = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_precharge_failed(error_flag value) {
  extern error_flag __oe_error_precharge_failed;
  if (__oe_error_precharge_failed != value) {
    __oe_error_precharge_failed = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_vdc_voltage(error_level value) {
  extern error_level __oe_error_level_vdc_voltage;
  if (__oe_error_level_vdc_voltage != value) {
    __oe_error_level_vdc_voltage = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_magnet_current_left(error_level value) {
  extern error_level __oe_error_level_magnet_current_left;
  if (__oe_error_level_magnet_current_left != value) {
    __oe_error_level_magnet_current_left = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_magnet_current_right(error_level value) {
  extern error_level __oe_error_level_magnet_current_right;
  if (__oe_error_level_magnet_current_right != value) {
    __oe_error_level_magnet_current_right = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_input_current(error_level value) {
  extern error_level __oe_error_level_input_current;
  if (__oe_error_level_input_current != value) {
    __oe_error_level_input_current = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_magnet_temperature_left(error_level value) {
  extern error_level __oe_error_level_magnet_temperature_left;
  if (__oe_error_level_magnet_temperature_left != value) {
    __oe_error_level_magnet_temperature_left = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_magnet_temperature_right(error_level value) {
  extern error_level __oe_error_level_magnet_temperature_right;
  if (__oe_error_level_magnet_temperature_right != value) {
    __oe_error_level_magnet_temperature_right = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_level_mcu_temperature(error_level value) {
  extern error_level __oe_error_level_mcu_temperature;
  if (__oe_error_level_mcu_temperature != value) {
    __oe_error_level_mcu_temperature = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_assertion_fault(error_flag value) {
  extern error_flag __oe_assertion_fault;
  if (__oe_assertion_fault != value) {
    __oe_assertion_fault = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_heartbeat_miss(error_flag value) {
  extern error_flag __oe_error_heartbeat_miss;
  if (__oe_error_heartbeat_miss != value) {
    __oe_error_heartbeat_miss = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
static uint32_t DMAMEM __oe_config_hash_send_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_build_time_send_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_magnet_current_send_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_input_current_send_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[7];
static uint32_t DMAMEM __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[7];
void canzero_send_config_hash() {
  canzero_message_get_resp msg;
  {
    uint64_t masked = (__oe_config_hash & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
    __oe_config_hash_send_fragmentation_buffer[0] = ((uint32_t*)&masked)[0];
    __oe_config_hash_send_fragmentation_buffer[1] = ((uint32_t*)&masked)[1];
  }
  msg.m_data = __oe_config_hash_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 0;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_config_hash_send_fragmentation_buffer, 2, 0, 255);

}
void canzero_send_build_time() {
  canzero_message_get_resp msg;
  __oe_build_time_send_fragmentation_buffer[0] = (__oe_build_time.m_year & (0xFFFFFFFF >> (32 - 16)));
  __oe_build_time_send_fragmentation_buffer[0] |= ((__oe_build_time.m_month & (0xFFFFFFFF >> (32 - 8))) << 16);
  __oe_build_time_send_fragmentation_buffer[0] |= ((__oe_build_time.m_day & (0xFFFFFFFF >> (32 - 8))) << 24);
  __oe_build_time_send_fragmentation_buffer[1] = (__oe_build_time.m_hour & (0xFFFFFFFF >> (32 - 8)));
  __oe_build_time_send_fragmentation_buffer[1] |= ((__oe_build_time.m_min & (0xFFFFFFFF >> (32 - 8))) << 8);
  __oe_build_time_send_fragmentation_buffer[1] |= ((__oe_build_time.m_sec & (0xFFFFFFFF >> (32 - 8))) << 16);

  msg.m_data = __oe_build_time_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 1;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_build_time_send_fragmentation_buffer, 2, 1, 255);

}
void canzero_send_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_state) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 2;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 3;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_command) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 4;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_precharge_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_precharge_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 5;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_feedthrough_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_feedthrough_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 6;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_control_active() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_control_active) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 7;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_arming_failed() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_arming_failed) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 8;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_precharge_failed() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_precharge_failed) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 9;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_outer_airgap_left() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_outer_airgap_left - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 10;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_inner_airgap_left() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_inner_airgap_left - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 11;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_outer_airgap_right() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_outer_airgap_right - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 12;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_inner_airgap_right() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_inner_airgap_right - (0)) / 0.00030518043793392844, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 13;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_vdc_voltage() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_vdc_voltage - (0)) / 0.0015259021896696422, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 14;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_vdc_voltage() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_vdc_voltage) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 15;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_config_vdc_voltage() {
  canzero_message_get_resp msg;
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_vdc_voltage.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_vdc_voltage.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_vdc_voltage.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_vdc_voltage.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_vdc_voltage.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_vdc_voltage.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[6] = (__oe_error_level_config_vdc_voltage.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[6] |= ((__oe_error_level_config_vdc_voltage.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
  __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[6] |= ((__oe_error_level_config_vdc_voltage.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

  msg.m_data = __oe_error_level_config_vdc_voltage_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 16;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_error_level_config_vdc_voltage_send_fragmentation_buffer, 7, 16, 255);

}
void canzero_send_current_left() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_current_left - (-25)) / 0.0007629510948348211, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 17;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_current_right() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_current_right - (-25)) / 0.0007629510948348211, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 18;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_input_current() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_input_current - (-25)) / 0.0007629510948348211, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 19;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_config_magnet_current() {
  canzero_message_get_resp msg;
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_magnet_current.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_magnet_current.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_magnet_current.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_magnet_current.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_magnet_current.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_magnet_current.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[6] = (__oe_error_level_config_magnet_current.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_current.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
  __oe_error_level_config_magnet_current_send_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_current.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

  msg.m_data = __oe_error_level_config_magnet_current_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 20;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_error_level_config_magnet_current_send_fragmentation_buffer, 7, 20, 255);

}
void canzero_send_error_level_magnet_current_left() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_current_left) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 21;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_magnet_current_right() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_current_right) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 22;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_config_input_current() {
  canzero_message_get_resp msg;
  __oe_error_level_config_input_current_send_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_input_current.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_input_current.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_input_current.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_input_current.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_input_current.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_input_current.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[6] = (__oe_error_level_config_input_current.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
  __oe_error_level_config_input_current_send_fragmentation_buffer[6] |= ((__oe_error_level_config_input_current.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
  __oe_error_level_config_input_current_send_fragmentation_buffer[6] |= ((__oe_error_level_config_input_current.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

  msg.m_data = __oe_error_level_config_input_current_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 23;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_error_level_config_input_current_send_fragmentation_buffer, 7, 23, 255);

}
void canzero_send_error_level_input_current() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_input_current) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 24;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_magnet_temperature_left1() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_magnet_temperature_left1 - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 25;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_magnet_temperature_left2() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_magnet_temperature_left2 - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 26;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_magnet_temperature_left_max() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_magnet_temperature_left_max - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 27;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_magnet_temperature_left() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_temperature_left) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 28;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_magnet_temperature_right1() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_magnet_temperature_right1 - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 29;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_magnet_temperature_right2() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_magnet_temperature_right2 - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 30;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_magnet_temperature_right_max() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_magnet_temperature_right_max - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 31;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_magnet_temperature_right() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_magnet_temperature_right) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 32;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_config_magnet_temperature() {
  canzero_message_get_resp msg;
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_magnet_temperature.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_magnet_temperature.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_magnet_temperature.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_magnet_temperature.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_magnet_temperature.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_magnet_temperature.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[6] = (__oe_error_level_config_magnet_temperature.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_temperature.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
  __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[6] |= ((__oe_error_level_config_magnet_temperature.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

  msg.m_data = __oe_error_level_config_magnet_temperature_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 33;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_error_level_config_magnet_temperature_send_fragmentation_buffer, 7, 33, 255);

}
void canzero_send_mcu_temperature() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_mcu_temperature - (-1)) / 0.592156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 34;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_mcu_temperature() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_level_mcu_temperature) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 35;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_level_config_mcu_temperature() {
  canzero_message_get_resp msg;
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[0] = (min_u32((__oe_error_level_config_mcu_temperature.m_info_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[1] = (min_u32((__oe_error_level_config_mcu_temperature.m_info_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[2] = (min_u32((__oe_error_level_config_mcu_temperature.m_warning_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[3] = (min_u32((__oe_error_level_config_mcu_temperature.m_warning_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[4] = (min_u32((__oe_error_level_config_mcu_temperature.m_error_thresh - ((float)-1000)) / (float)0.0000004656612874161595, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[5] = (min_u32((__oe_error_level_config_mcu_temperature.m_error_timeout - ((float)0)) / (float)0.000000013969838622484784, 0xFFFFFFFFul) & (0xFFFFFFFF >> (32 - 32)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[6] = (__oe_error_level_config_mcu_temperature.m_ignore_info & (0xFFFFFFFF >> (32 - 1)));
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[6] |= ((__oe_error_level_config_mcu_temperature.m_ignore_warning & (0xFFFFFFFF >> (32 - 1))) << 1);
  __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[6] |= ((__oe_error_level_config_mcu_temperature.m_ignore_error & (0xFFFFFFFF >> (32 - 1))) << 2);

  msg.m_data = __oe_error_level_config_mcu_temperature_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 36;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_error_level_config_mcu_temperature_send_fragmentation_buffer, 7, 36, 255);

}
void canzero_send_loop_frequency() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_loop_frequency - (0)) / 0.00000023283064370807974, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 37;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_assertion_fault() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_assertion_fault) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 38;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_heartbeat_miss() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_heartbeat_miss) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 39;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_lt2() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_lt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 40;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_rt2() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_rt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 41;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_lsb_x() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_lsb_x - (-1)) / 0.00784313725490196, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 42;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_lsb_y() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_lsb_y - (-1)) / 0.00784313725490196, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 43;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_rsb_x() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_rsb_x - (-1)) / 0.00784313725490196, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 44;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_rsb_y() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_rsb_y - (-1)) / 0.00784313725490196, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 45;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_lt1_down() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_gamepad_lt1_down) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 46;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_rt1_down() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_gamepad_rt1_down) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 47;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_x_down() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_gamepad_x_down) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 48;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_guidance_board_front;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
