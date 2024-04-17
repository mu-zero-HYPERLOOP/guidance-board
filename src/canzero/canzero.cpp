#include "canzero.h"
mgu_state __oe_state;
mgu_command __oe_command;
mgu_pid_values __oe_control_config;
sdc_status __oe_sdc_status;
float __oe_air_gap_starboard;
float __oe_air_gap_port;
float __oe_target_force_starboard;
float __oe_target_force_port;
float __oe_dc_current_starboard;
float __oe_dc_current_port;
float __oe_power_estimation;
float __oe_magnet_temperature_starboard;
float __oe_magnet_temperature_port;
float __oe_mcu_temperature;
float __oe_mosfet_temperature;
error_flag __oe_error_45V_over_voltage;
error_flag __oe_error_45V_under_voltage;
error_flag __oe_error_control_error;
error_flag __oe_error_magnet_over_temperature_starboard;
error_flag __oe_error_magnet_over_temperature_port;
error_flag __oe_error_mosfet_over_temperature;
error_flag __oe_error_mcu_over_temperature;
error_flag __oe_warn_magnet_over_temperature_starboard;
error_flag __oe_warn_magnet_over_temperature_port;
error_flag __oe_warn_mosfet_over_temperature;
error_flag __oe_warn_mcu_over_temperature;

typedef enum {
  HEARTBEAT_JOB_TAG = 0,
  GET_RESP_FRAGMENTATION_JOB_TAG = 1,
  STREAM_INTERVAL_JOB_TAG = 2,
} job_tag;
typedef struct {
  uint32_t *buffer;
  uint8_t offset;
  uint8_t size;
  uint8_t od_index;
  uint8_t server_id;
} get_resp_fragmentation_job;
typedef struct {
  uint32_t command_resp_msg_id;
  uint8_t bus_id;
} command_resp_timeout_job;
typedef struct {
  uint32_t last_schedule; 
  uint32_t stream_id;
} stream_interval_job;
typedef struct {
  uint32_t climax;
  uint32_t position;
  job_tag tag;
  union {
    get_resp_fragmentation_job get_fragmentation_job;
    stream_interval_job stream_interval_job;
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
static job_pool_allocator job_allocator;
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
#define SCHEDULE_HEAP_SIZE 256
typedef struct {
  job_t *heap[SCHEDULE_HEAP_SIZE]; // job**
  uint32_t size;
} job_scheduler_t;
static job_scheduler_t scheduler;
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
  if (scheduler.size >= SCHEDULE_HEAP_SIZE) {
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
static void schedule_get_resp_fragmentation_job(uint32_t *fragmentation_buffer, uint8_t size, uint8_t od_index, uint8_t server_id) {
  job_t *fragmentation_job = job_pool_allocator_alloc();
  fragmentation_job->climax = canzero_get_time() + get_resp_fragmentation_interval;
  fragmentation_job->tag = GET_RESP_FRAGMENTATION_JOB_TAG;
  fragmentation_job->job.get_fragmentation_job.buffer = fragmentation_buffer;
  fragmentation_job->job.get_fragmentation_job.offset = 1;
  fragmentation_job->job.get_fragmentation_job.size = size;
  fragmentation_job->job.get_fragmentation_job.od_index = od_index;
  fragmentation_job->job.get_fragmentation_job.server_id = server_id;
  scheduler_schedule(fragmentation_job);
}
static job_t heartbeat_job;
static const uint32_t heartbeat_interval = 100;
static void schedule_heartbeat_job() {
  heartbeat_job.climax = canzero_get_time() + heartbeat_interval;
  heartbeat_job.tag = HEARTBEAT_JOB_TAG;
  scheduler_schedule(&heartbeat_job);
}
static job_t state_interval_job;
static const uint32_t state_interval = 0;
static void schedule_state_interval_job(){
  uint32_t time = canzero_get_time();
  state_interval_job.climax = time + state_interval;
  state_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  state_interval_job.job.stream_interval_job.stream_id = 0;
  state_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&state_interval_job);
}
static job_t air_gaps_interval_job;
static const uint32_t air_gaps_interval = 100;
static void schedule_air_gaps_interval_job(){
  uint32_t time = canzero_get_time();
  air_gaps_interval_job.climax = time + air_gaps_interval;
  air_gaps_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  air_gaps_interval_job.job.stream_interval_job.stream_id = 1;
  air_gaps_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&air_gaps_interval_job);
}
static job_t power_estimation_interval_job;
static const uint32_t power_estimation_interval = 1000;
static void schedule_power_estimation_interval_job(){
  uint32_t time = canzero_get_time();
  power_estimation_interval_job.climax = time + power_estimation_interval;
  power_estimation_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  power_estimation_interval_job.job.stream_interval_job.stream_id = 2;
  power_estimation_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&power_estimation_interval_job);
}
static job_t temperatures_interval_job;
static const uint32_t temperatures_interval = 2500;
static void schedule_temperatures_interval_job(){
  uint32_t time = canzero_get_time();
  temperatures_interval_job.climax = time + temperatures_interval;
  temperatures_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  temperatures_interval_job.job.stream_interval_job.stream_id = 3;
  temperatures_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&temperatures_interval_job);
}
static job_t sdc_status_interval_job;
static const uint32_t sdc_status_interval = 0;
static void schedule_sdc_status_interval_job(){
  uint32_t time = canzero_get_time();
  sdc_status_interval_job.climax = time + sdc_status_interval;
  sdc_status_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  sdc_status_interval_job.job.stream_interval_job.stream_id = 4;
  sdc_status_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&sdc_status_interval_job);
}
static job_t errors_interval_job;
static const uint32_t errors_interval = 0;
static void schedule_errors_interval_job(){
  uint32_t time = canzero_get_time();
  errors_interval_job.climax = time + errors_interval;
  errors_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  errors_interval_job.job.stream_interval_job.stream_id = 5;
  errors_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&errors_interval_job);
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
      switch (job->job.stream_interval_job.stream_id) {
      case 0: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mgu1_stream_state stream_message;
        stream_message.state = __oe_state;
        stream_message.command = __oe_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mgu1_stream_state(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 1: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 100);
        canzero_exit_critical();
        canzero_message_mgu1_stream_air_gaps stream_message;
        stream_message.air_gap_starboard = __oe_air_gap_starboard;
        stream_message.air_gap_port = __oe_air_gap_port;
        stream_message.target_force_starboard = __oe_target_force_starboard;
        stream_message.target_force_port = __oe_target_force_port;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mgu1_stream_air_gaps(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 2: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 1000);
        canzero_exit_critical();
        canzero_message_mgu1_stream_power_estimation stream_message;
        stream_message.dc_current_starboard = __oe_dc_current_starboard;
        stream_message.dc_current_port = __oe_dc_current_port;
        stream_message.power_estimation = __oe_power_estimation;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mgu1_stream_power_estimation(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 3: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 2500);
        canzero_exit_critical();
        canzero_message_mgu1_stream_temperatures stream_message;
        stream_message.magnet_temperature_starboard = __oe_magnet_temperature_starboard;
        stream_message.magnet_temperature_port = __oe_magnet_temperature_port;
        stream_message.mcu_temperature = __oe_mcu_temperature;
        stream_message.mosfet_temperature = __oe_mosfet_temperature;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mgu1_stream_temperatures(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 4: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 1000);
        canzero_exit_critical();
        canzero_message_mgu1_stream_sdc_status stream_message;
        stream_message.sdc_status = __oe_sdc_status;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mgu1_stream_sdc_status(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 5: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 1000);
        canzero_exit_critical();
        canzero_message_mgu1_stream_errors stream_message;
        stream_message.error_45V_over_voltage = __oe_error_45V_over_voltage;
        stream_message.error_45V_under_voltage = __oe_error_45V_under_voltage;
        stream_message.error_control_error = __oe_error_control_error;
        stream_message.error_magnet_over_temperature_starboard = __oe_error_magnet_over_temperature_starboard;
        stream_message.error_magnet_over_temperature_port = __oe_error_magnet_over_temperature_port;
        stream_message.error_mosfet_over_temperature = __oe_error_mosfet_over_temperature;
        stream_message.error_mcu_over_temperature = __oe_error_mcu_over_temperature;
        stream_message.warn_magnet_over_temperature_starboard = __oe_warn_magnet_over_temperature_starboard;
        stream_message.warn_magnet_over_temperature_port = __oe_warn_magnet_over_temperature_port;
        stream_message.warn_mosfet_over_temperature = __oe_warn_mosfet_over_temperature;
        stream_message.warn_mcu_over_temperature = __oe_warn_mcu_over_temperature;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mgu1_stream_errors(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
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
      canzero_message_heartbeat heartbeat;
      heartbeat.node_id = node_id_mgu1;
      canzero_frame heartbeat_frame;
      canzero_serialize_canzero_message_heartbeat(&heartbeat, &heartbeat_frame);
      canzero_can0_send(&heartbeat_frame);
      break;
    }
    case GET_RESP_FRAGMENTATION_JOB_TAG: {
      get_resp_fragmentation_job *fragmentation_job = &job->job.get_fragmentation_job;
      canzero_message_get_resp fragmentation_response;
      fragmentation_response.header.sof = 0;
      fragmentation_response.header.toggle = fragmentation_job->offset % 2;
      fragmentation_response.header.od_index = fragmentation_job->od_index;
      fragmentation_response.header.client_id = 0x7;
      fragmentation_response.header.server_id = fragmentation_job->server_id;
      fragmentation_response.data = fragmentation_job->buffer[fragmentation_job->offset];
      fragmentation_job->offset += 1;
      if (fragmentation_job->offset == fragmentation_job->size) {
        fragmentation_response.header.eof = 1;
        scheduler_unschedule();
      } else {
        fragmentation_response.header.eof = 0;
        scheduler_reschedule(time + get_resp_fragmentation_interval);
      }
      canzero_exit_critical();
      canzero_frame fragmentation_frame;
      canzero_serialize_canzero_message_get_resp(&fragmentation_response, &fragmentation_frame);
      canzero_can1_send(&fragmentation_frame);
      break;
    }
    default:
      canzero_exit_critical();
      break;
    }
  }
}
static uint32_t scheduler_next_job_timeout(){
  return scheduler.heap[0]->climax;
}
static uint32_t __oe_control_config_rx_fragmentation_buffer[2];
static void canzero_handle_get_req(canzero_frame* frame) {
  canzero_message_get_req msg;
  canzero_deserialize_canzero_message_get_req(frame, &msg);
  if (msg.header.server_id != 7) {
    return;
  }
  canzero_message_get_resp resp;
  switch (msg.header.od_index) {
  case 0: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 1: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_command) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 2: {
    __oe_control_config_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_control_config.p_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));
    __oe_control_config_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_control_config.i_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));
    __oe_control_config_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_control_config.d_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));

    resp.data = __oe_control_config_rx_fragmentation_buffer[0];
    resp.header.sof = 1;
    resp.header.eof = 0;
    resp.header.toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_control_config_rx_fragmentation_buffer, 2, 2, msg.header.server_id);
    break;
  }
  case 3: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 4: {
    resp.data |= ((uint32_t)((__oe_air_gap_starboard - (0)) / 0.00030518043793392844)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 5: {
    resp.data |= ((uint32_t)((__oe_air_gap_port - (0)) / 0.00030518043793392844)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 6: {
    resp.data |= ((uint32_t)((__oe_target_force_starboard - (0)) / 0.0015259021896696422)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 7: {
    resp.data |= ((uint32_t)((__oe_target_force_port - (0)) / 0.0015259021896696422)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 8: {
    resp.data |= ((uint32_t)((__oe_dc_current_starboard - (-10)) / 0.0009155413138017853)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 9: {
    resp.data |= ((uint32_t)((__oe_dc_current_port - (-10)) / 0.0009155413138017853)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 10: {
    resp.data |= ((uint32_t)((__oe_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 11: {
    resp.data |= ((uint32_t)((__oe_magnet_temperature_starboard - (-1)) / 0.592156862745098)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 12: {
    resp.data |= ((uint32_t)((__oe_magnet_temperature_port - (-1)) / 0.592156862745098)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 13: {
    resp.data |= ((uint32_t)((__oe_mcu_temperature - (-1)) / 0.592156862745098)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 14: {
    resp.data |= ((uint32_t)((__oe_mosfet_temperature - (-1)) / 0.592156862745098)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 15: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 16: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 17: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_control_error) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 18: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_magnet_over_temperature_starboard) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 19: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_magnet_over_temperature_port) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 20: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 21: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 22: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_magnet_over_temperature_starboard) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 23: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_magnet_over_temperature_port) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 24: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 25: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  }
  resp.header.od_index = msg.header.od_index;
  resp.header.client_id = msg.header.client_id;
  resp.header.server_id = msg.header.server_id;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_get_resp(&resp, &resp_frame);
  canzero_can1_send(&resp_frame);
}
static uint32_t control_config_tmp_tx_fragmentation_buffer[2];
static uint32_t control_config_tmp_tx_fragmentation_offset = 0;
static void canzero_handle_set_req(canzero_frame* frame) {
  canzero_message_set_req msg;
  canzero_deserialize_canzero_message_set_req(frame, &msg);
  if (msg.header.server_id != 7) {
    return;
  }
  canzero_message_set_resp resp;
  switch (msg.header.od_index) {
  case 0 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mgu_state state_tmp;
    state_tmp = (mgu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_state(state_tmp);
    break;
  }
  case 1 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mgu_command command_tmp;
    command_tmp = (mgu_command)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_command(command_tmp);
    break;
  }
  case 2 : {
    if (msg.header.sof == 1) {
      if (msg.header.toggle == 0 || msg.header.eof != 0) {
        return;
      }
      control_config_tmp_tx_fragmentation_offset = 0;
    }else {
      control_config_tmp_tx_fragmentation_offset += 1;
      if (control_config_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    control_config_tmp_tx_fragmentation_buffer[control_config_tmp_tx_fragmentation_offset] = msg.data;
    if (msg.header.eof == 0) {
      return;
    }
    mgu_pid_values control_config_tmp;
    control_config_tmp.p_value = ((control_config_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    control_config_tmp.i_value = ((control_config_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    control_config_tmp.d_value = ((control_config_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    canzero_set_control_config(control_config_tmp);
    break;
  }
  case 3 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status sdc_status_tmp;
    sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_sdc_status(sdc_status_tmp);
    break;
  }
  case 4 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float air_gap_starboard_tmp;
    air_gap_starboard_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0);
    canzero_set_air_gap_starboard(air_gap_starboard_tmp);
    break;
  }
  case 5 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float air_gap_port_tmp;
    air_gap_port_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0);
    canzero_set_air_gap_port(air_gap_port_tmp);
    break;
  }
  case 6 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float target_force_starboard_tmp;
    target_force_starboard_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + 0);
    canzero_set_target_force_starboard(target_force_starboard_tmp);
    break;
  }
  case 7 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float target_force_port_tmp;
    target_force_port_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + 0);
    canzero_set_target_force_port(target_force_port_tmp);
    break;
  }
  case 8 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float dc_current_starboard_tmp;
    dc_current_starboard_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10);
    canzero_set_dc_current_starboard(dc_current_starboard_tmp);
    break;
  }
  case 9 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float dc_current_port_tmp;
    dc_current_port_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10);
    canzero_set_dc_current_port(dc_current_port_tmp);
    break;
  }
  case 10 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float power_estimation_tmp;
    power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_power_estimation(power_estimation_tmp);
    break;
  }
  case 11 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float magnet_temperature_starboard_tmp;
    magnet_temperature_starboard_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_starboard(magnet_temperature_starboard_tmp);
    break;
  }
  case 12 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float magnet_temperature_port_tmp;
    magnet_temperature_port_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_magnet_temperature_port(magnet_temperature_port_tmp);
    break;
  }
  case 13 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mcu_temperature_tmp;
    mcu_temperature_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_mcu_temperature(mcu_temperature_tmp);
    break;
  }
  case 14 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mosfet_temperature_tmp;
    mosfet_temperature_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_mosfet_temperature(mosfet_temperature_tmp);
    break;
  }
  case 15 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_45V_over_voltage_tmp;
    error_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_45V_over_voltage(error_45V_over_voltage_tmp);
    break;
  }
  case 16 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_45V_under_voltage_tmp;
    error_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_45V_under_voltage(error_45V_under_voltage_tmp);
    break;
  }
  case 17 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_control_error_tmp;
    error_control_error_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_control_error(error_control_error_tmp);
    break;
  }
  case 18 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_magnet_over_temperature_starboard_tmp;
    error_magnet_over_temperature_starboard_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_magnet_over_temperature_starboard(error_magnet_over_temperature_starboard_tmp);
    break;
  }
  case 19 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_magnet_over_temperature_port_tmp;
    error_magnet_over_temperature_port_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_magnet_over_temperature_port(error_magnet_over_temperature_port_tmp);
    break;
  }
  case 20 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mosfet_over_temperature_tmp;
    error_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mosfet_over_temperature(error_mosfet_over_temperature_tmp);
    break;
  }
  case 21 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mcu_over_temperature_tmp;
    error_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mcu_over_temperature(error_mcu_over_temperature_tmp);
    break;
  }
  case 22 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_magnet_over_temperature_starboard_tmp;
    warn_magnet_over_temperature_starboard_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_magnet_over_temperature_starboard(warn_magnet_over_temperature_starboard_tmp);
    break;
  }
  case 23 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_magnet_over_temperature_port_tmp;
    warn_magnet_over_temperature_port_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_magnet_over_temperature_port(warn_magnet_over_temperature_port_tmp);
    break;
  }
  case 24 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mosfet_over_temperature_tmp;
    warn_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mosfet_over_temperature(warn_mosfet_over_temperature_tmp);
    break;
  }
  case 25 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mcu_over_temperature_tmp;
    warn_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mcu_over_temperature(warn_mcu_over_temperature_tmp);
    break;
  }
  default:
    return;
  }
  resp.header.od_index = msg.header.od_index;
  resp.header.client_id = msg.header.client_id;
  resp.header.server_id = msg.header.server_id;
  resp.header.erno = set_resp_erno_Success;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_set_resp(&resp, &resp_frame);
  canzero_can1_send(&resp_frame);

}
static void canzero_handle_master_stream_mgu_control(canzero_frame* frame) {
  canzero_message_master_stream_mgu_control msg;
  canzero_deserialize_canzero_message_master_stream_mgu_control(frame, &msg);
  canzero_set_command(msg.mgu_command);
}
__attribute__((weak)) void canzero_handle_heartbeat(canzero_frame* frame) {
  canzero_message_heartbeat msg;
  canzero_deserialize_canzero_message_heartbeat(frame, &msg);
}
void canzero_can0_poll() {
  canzero_frame frame;
  while (canzero_can0_recv(&frame)) {
    switch (frame.id) {
      case 0x13B:
        canzero_handle_get_req(&frame);
        break;
      case 0x15B:
        canzero_handle_set_req(&frame);
        break;
      case 0x17B:
        canzero_handle_heartbeat(&frame);
        break;
    }
  }
}
void canzero_can1_poll() {
  canzero_frame frame;
  while (canzero_can1_recv(&frame)) {
    switch (frame.id) {
      case 0x53:
        canzero_handle_master_stream_mgu_control(&frame);
        break;
    }
  }
}
uint32_t canzero_update_continue(uint32_t time){
  schedule_jobs(time);
  return scheduler_next_job_timeout();
}
void canzero_init() {
  canzero_can0_setup(1000000, NULL, 0);
  canzero_can1_setup(1000000, NULL, 0);

  job_pool_allocator_init();
  scheduler.size = 0;
  schedule_heartbeat_job();
  schedule_state_interval_job();
  schedule_air_gaps_interval_job();
  schedule_power_estimation_interval_job();
  schedule_temperatures_interval_job();
  schedule_sdc_status_interval_job();
  schedule_errors_interval_job();

}
void canzero_set_state(mgu_state value) {
  extern mgu_state __oe_state;
  if (__oe_state != value) {
    __oe_state = value;
    uint32_t time = canzero_get_time();
    if (state_interval_job.climax > state_interval_job.job.stream_interval_job.last_schedule + 0) {
      state_interval_job.climax = state_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_command(mgu_command value) {
  extern mgu_command __oe_command;
  if (__oe_command != value) {
    __oe_command = value;
    uint32_t time = canzero_get_time();
    if (state_interval_job.climax > state_interval_job.job.stream_interval_job.last_schedule + 0) {
      state_interval_job.climax = state_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_sdc_status(sdc_status value) {
  extern sdc_status __oe_sdc_status;
  if (__oe_sdc_status != value) {
    __oe_sdc_status = value;
    uint32_t time = canzero_get_time();
    if (sdc_status_interval_job.climax > sdc_status_interval_job.job.stream_interval_job.last_schedule + 0) {
      sdc_status_interval_job.climax = sdc_status_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&sdc_status_interval_job);
    }
  }
}
void canzero_set_error_45V_over_voltage(error_flag value) {
  extern error_flag __oe_error_45V_over_voltage;
  if (__oe_error_45V_over_voltage != value) {
    __oe_error_45V_over_voltage = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_45V_under_voltage(error_flag value) {
  extern error_flag __oe_error_45V_under_voltage;
  if (__oe_error_45V_under_voltage != value) {
    __oe_error_45V_under_voltage = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_control_error(error_flag value) {
  extern error_flag __oe_error_control_error;
  if (__oe_error_control_error != value) {
    __oe_error_control_error = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_magnet_over_temperature_starboard(error_flag value) {
  extern error_flag __oe_error_magnet_over_temperature_starboard;
  if (__oe_error_magnet_over_temperature_starboard != value) {
    __oe_error_magnet_over_temperature_starboard = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_magnet_over_temperature_port(error_flag value) {
  extern error_flag __oe_error_magnet_over_temperature_port;
  if (__oe_error_magnet_over_temperature_port != value) {
    __oe_error_magnet_over_temperature_port = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_mosfet_over_temperature(error_flag value) {
  extern error_flag __oe_error_mosfet_over_temperature;
  if (__oe_error_mosfet_over_temperature != value) {
    __oe_error_mosfet_over_temperature = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_error_mcu_over_temperature(error_flag value) {
  extern error_flag __oe_error_mcu_over_temperature;
  if (__oe_error_mcu_over_temperature != value) {
    __oe_error_mcu_over_temperature = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_warn_magnet_over_temperature_starboard(error_flag value) {
  extern error_flag __oe_warn_magnet_over_temperature_starboard;
  if (__oe_warn_magnet_over_temperature_starboard != value) {
    __oe_warn_magnet_over_temperature_starboard = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_warn_magnet_over_temperature_port(error_flag value) {
  extern error_flag __oe_warn_magnet_over_temperature_port;
  if (__oe_warn_magnet_over_temperature_port != value) {
    __oe_warn_magnet_over_temperature_port = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_warn_mosfet_over_temperature(error_flag value) {
  extern error_flag __oe_warn_mosfet_over_temperature;
  if (__oe_warn_mosfet_over_temperature != value) {
    __oe_warn_mosfet_over_temperature = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
void canzero_set_warn_mcu_over_temperature(error_flag value) {
  extern error_flag __oe_warn_mcu_over_temperature;
  if (__oe_warn_mcu_over_temperature != value) {
    __oe_warn_mcu_over_temperature = value;
    uint32_t time = canzero_get_time();
    if (errors_interval_job.climax > errors_interval_job.job.stream_interval_job.last_schedule + 0) {
      errors_interval_job.climax = errors_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
