#ifndef CANZERO_H
#define CANZERO_H
#include "inttypes.h"
#include "stddef.h"
typedef struct {
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} get_req_header;
typedef struct {
  uint8_t sof;
  uint8_t eof;
  uint8_t toggle;
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} set_req_header;
typedef enum {
  mgu_command_NONE = 0,
  mgu_command_PRECHARGE = 1,
  mgu_command_START = 2,
  mgu_command_STOP = 3,
  mgu_command_DISCONNECT = 4,
} mgu_command;
typedef enum {
  node_id_mlu1 = 0,
  node_id_master = 1,
  node_id_mlu2 = 2,
  node_id_mlu3 = 3,
  node_id_mlu4 = 4,
  node_id_mlu5 = 5,
  node_id_mlu6 = 6,
  node_id_mgu1 = 7,
  node_id_mgu2 = 8,
  node_id_motor_driver = 9,
  node_id_pdu24 = 10,
  node_id_pdu12 = 11,
  node_id_input_board = 12,
  node_id_gamepad = 13,
} node_id;
typedef struct {
  uint8_t sof;
  uint8_t eof;
  uint8_t toggle;
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} get_resp_header;
typedef enum {
  set_resp_erno_Success = 0,
  set_resp_erno_Error = 1,
} set_resp_erno;
typedef struct {
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
  set_resp_erno erno;
} set_resp_header;
typedef enum {
  mgu_state_INIT = 0,
  mgu_state_IDLE = 1,
  mgu_state_PRECHARGE = 2,
  mgu_state_READY = 3,
  mgu_state_START = 4,
  mgu_state_CONTROL = 5,
  mgu_state_STOP = 6,
} mgu_state;
typedef enum {
  sdc_status_OPEN = 0,
  sdc_status_CLOSED = 1,
} sdc_status;
typedef enum {
  error_flag_OK = 0,
  error_flag_ERROR = 1,
} error_flag;
typedef struct {
  float p_value;
  float i_value;
  float d_value;
} mgu_pid_values;
typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} canzero_frame;
typedef enum : uint32_t {
  CANZERO_FRAME_IDE_BIT = 0x40000000, // 1 << 30
  CANZERO_FRAME_RTR_BIT = 0x80000000, // 1 << 31
} can_frame_id_bits;
typedef struct {
  uint32_t mask;
  uint32_t id;
} canzero_can_filter;
extern void canzero_can0_setup(uint32_t baudrate, canzero_can_filter* filters, int filter_count);
extern void canzero_can0_send(canzero_frame* frame);
extern int canzero_can0_recv(canzero_frame* frame);
extern void canzero_can1_setup(uint32_t baudrate, canzero_can_filter* filters, int filter_count);
extern void canzero_can1_send(canzero_frame* frame);
extern int canzero_can1_recv(canzero_frame* frame);
extern void canzero_request_update(uint32_t time);
extern uint32_t canzero_get_time();
extern void canzero_enter_critical();
extern void canzero_exit_critical();
static inline mgu_state canzero_get_state() {
  extern mgu_state __oe_state;
  return __oe_state;
}
static inline mgu_command canzero_get_command() {
  extern mgu_command __oe_command;
  return __oe_command;
}
static inline mgu_pid_values canzero_get_control_config() {
  extern mgu_pid_values __oe_control_config;
  return __oe_control_config;
}
static inline sdc_status canzero_get_sdc_status() {
  extern sdc_status __oe_sdc_status;
  return __oe_sdc_status;
}
static inline float canzero_get_air_gap_starboard() {
  extern float __oe_air_gap_starboard;
  return __oe_air_gap_starboard;
}
static inline float canzero_get_air_gap_port() {
  extern float __oe_air_gap_port;
  return __oe_air_gap_port;
}
static inline float canzero_get_target_force_starboard() {
  extern float __oe_target_force_starboard;
  return __oe_target_force_starboard;
}
static inline float canzero_get_target_force_port() {
  extern float __oe_target_force_port;
  return __oe_target_force_port;
}
static inline float canzero_get_dc_current_starboard() {
  extern float __oe_dc_current_starboard;
  return __oe_dc_current_starboard;
}
static inline float canzero_get_dc_current_port() {
  extern float __oe_dc_current_port;
  return __oe_dc_current_port;
}
static inline float canzero_get_power_estimation() {
  extern float __oe_power_estimation;
  return __oe_power_estimation;
}
static inline float canzero_get_magnet_temperature_starboard() {
  extern float __oe_magnet_temperature_starboard;
  return __oe_magnet_temperature_starboard;
}
static inline float canzero_get_magnet_temperature_port() {
  extern float __oe_magnet_temperature_port;
  return __oe_magnet_temperature_port;
}
static inline float canzero_get_mcu_temperature() {
  extern float __oe_mcu_temperature;
  return __oe_mcu_temperature;
}
static inline float canzero_get_mosfet_temperature() {
  extern float __oe_mosfet_temperature;
  return __oe_mosfet_temperature;
}
static inline error_flag canzero_get_error_45V_over_voltage() {
  extern error_flag __oe_error_45V_over_voltage;
  return __oe_error_45V_over_voltage;
}
static inline error_flag canzero_get_error_45V_under_voltage() {
  extern error_flag __oe_error_45V_under_voltage;
  return __oe_error_45V_under_voltage;
}
static inline error_flag canzero_get_error_control_error() {
  extern error_flag __oe_error_control_error;
  return __oe_error_control_error;
}
static inline error_flag canzero_get_error_magnet_over_temperature_starboard() {
  extern error_flag __oe_error_magnet_over_temperature_starboard;
  return __oe_error_magnet_over_temperature_starboard;
}
static inline error_flag canzero_get_error_magnet_over_temperature_port() {
  extern error_flag __oe_error_magnet_over_temperature_port;
  return __oe_error_magnet_over_temperature_port;
}
static inline error_flag canzero_get_error_mosfet_over_temperature() {
  extern error_flag __oe_error_mosfet_over_temperature;
  return __oe_error_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mcu_over_temperature() {
  extern error_flag __oe_error_mcu_over_temperature;
  return __oe_error_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_magnet_over_temperature_starboard() {
  extern error_flag __oe_warn_magnet_over_temperature_starboard;
  return __oe_warn_magnet_over_temperature_starboard;
}
static inline error_flag canzero_get_warn_magnet_over_temperature_port() {
  extern error_flag __oe_warn_magnet_over_temperature_port;
  return __oe_warn_magnet_over_temperature_port;
}
static inline error_flag canzero_get_warn_mosfet_over_temperature() {
  extern error_flag __oe_warn_mosfet_over_temperature;
  return __oe_warn_mosfet_over_temperature;
}
static inline error_flag canzero_get_warn_mcu_over_temperature() {
  extern error_flag __oe_warn_mcu_over_temperature;
  return __oe_warn_mcu_over_temperature;
}
typedef struct {
  get_resp_header header;
  uint32_t data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0x13F;
typedef struct {
  set_resp_header header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0x15F;
typedef struct {
  mgu_state state;
  mgu_command command;
} canzero_message_mgu1_stream_state;
static const uint32_t canzero_message_mgu1_stream_state_id = 0x59;
typedef struct {
  float air_gap_starboard;
  float air_gap_port;
  float target_force_starboard;
  float target_force_port;
} canzero_message_mgu1_stream_air_gaps;
static const uint32_t canzero_message_mgu1_stream_air_gaps_id = 0xDF;
typedef struct {
  float dc_current_starboard;
  float dc_current_port;
  float power_estimation;
} canzero_message_mgu1_stream_power_estimation;
static const uint32_t canzero_message_mgu1_stream_power_estimation_id = 0xFA;
typedef struct {
  float magnet_temperature_starboard;
  float magnet_temperature_port;
  float mcu_temperature;
  float mosfet_temperature;
} canzero_message_mgu1_stream_temperatures;
static const uint32_t canzero_message_mgu1_stream_temperatures_id = 0xFF;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mgu1_stream_sdc_status;
static const uint32_t canzero_message_mgu1_stream_sdc_status_id = 0x11A;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_control_error;
  error_flag error_magnet_over_temperature_starboard;
  error_flag error_magnet_over_temperature_port;
  error_flag error_mosfet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag warn_magnet_over_temperature_starboard;
  error_flag warn_magnet_over_temperature_port;
  error_flag warn_mosfet_over_temperature;
  error_flag warn_mcu_over_temperature;
} canzero_message_mgu1_stream_errors;
static const uint32_t canzero_message_mgu1_stream_errors_id = 0xDA;
typedef struct {
  node_id node_id;
} canzero_message_heartbeat;
static const uint32_t canzero_message_heartbeat_id = 0x17B;
typedef struct {
  get_req_header header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0x13B;
typedef struct {
  set_req_header header;
  uint32_t data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0x15B;
typedef struct {
  mgu_command mgu_command;
} canzero_message_master_stream_mgu_control;
static const uint32_t canzero_message_master_stream_mgu_control_id = 0x53;
static void canzero_serialize_canzero_message_get_resp(canzero_message_get_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x13F;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->header.sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->header.client_id << 16;
  ((uint32_t*)data)[0] |= msg->header.server_id << 24;
  ((uint32_t*)data)[1] = msg->data;
}
static void canzero_serialize_canzero_message_set_resp(canzero_message_set_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x15F;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->header.client_id << 13;
  ((uint32_t*)data)[0] |= msg->header.server_id << 21;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.erno & (0xFF >> (8 - 1))) << 29;
}
static void canzero_serialize_canzero_message_mgu1_stream_state(canzero_message_mgu1_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x59;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mgu1_stream_air_gaps(canzero_message_mgu1_stream_air_gaps* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xDF;
  frame->dlc = 8;
  float air_gap_starboard_0 = (msg->air_gap_starboard - 0) / 0.00030518043793392844;
  if (air_gap_starboard_0 > 4294901760.000) {
    air_gap_starboard_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) air_gap_starboard_0;
  float air_gap_port_16 = (msg->air_gap_port - 0) / 0.00030518043793392844;
  if (air_gap_port_16 > 4294901760.000) {
    air_gap_port_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) air_gap_port_16 << 16;
  float target_force_starboard_32 = (msg->target_force_starboard - 0) / 0.0015259021896696422;
  if (target_force_starboard_32 > 4294901760.000) {
    target_force_starboard_32 = 4294901760.000;
  }
  ((uint32_t*)data)[1] = (uint32_t) target_force_starboard_32;
  float target_force_port_48 = (msg->target_force_port - 0) / 0.0015259021896696422;
  if (target_force_port_48 > 4294901760.000) {
    target_force_port_48 = 4294901760.000;
  }
  ((uint32_t*)data)[1] |= (uint32_t) target_force_port_48 << 16;
}
static void canzero_serialize_canzero_message_mgu1_stream_power_estimation(canzero_message_mgu1_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xFA;
  frame->dlc = 6;
  float dc_current_starboard_0 = (msg->dc_current_starboard - -10) / 0.0009155413138017853;
  if (dc_current_starboard_0 > 4294901760.000) {
    dc_current_starboard_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_starboard_0;
  float dc_current_port_16 = (msg->dc_current_port - -10) / 0.0009155413138017853;
  if (dc_current_port_16 > 4294901760.000) {
    dc_current_port_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) dc_current_port_16 << 16;
  float power_estimation_32 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_32 > 4294901760.000) {
    power_estimation_32 = 4294901760.000;
  }
  ((uint32_t*)data)[1] = (uint32_t) power_estimation_32;
}
static void canzero_serialize_canzero_message_mgu1_stream_temperatures(canzero_message_mgu1_stream_temperatures* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xFF;
  frame->dlc = 4;
  float magnet_temperature_starboard_0 = (msg->magnet_temperature_starboard - -1) / 0.592156862745098;
  if (magnet_temperature_starboard_0 > 4278190080.000) {
    magnet_temperature_starboard_0 = 4278190080.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) magnet_temperature_starboard_0;
  float magnet_temperature_port_8 = (msg->magnet_temperature_port - -1) / 0.592156862745098;
  if (magnet_temperature_port_8 > 4278190080.000) {
    magnet_temperature_port_8 = 4278190080.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) magnet_temperature_port_8 << 8;
  float mcu_temperature_16 = (msg->mcu_temperature - -1) / 0.592156862745098;
  if (mcu_temperature_16 > 4278190080.000) {
    mcu_temperature_16 = 4278190080.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) mcu_temperature_16 << 16;
  float mosfet_temperature_24 = (msg->mosfet_temperature - -1) / 0.592156862745098;
  if (mosfet_temperature_24 > 4278190080.000) {
    mosfet_temperature_24 = 4278190080.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) mosfet_temperature_24 << 24;
}
static void canzero_serialize_canzero_message_mgu1_stream_sdc_status(canzero_message_mgu1_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x11A;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mgu1_stream_errors(canzero_message_mgu1_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xDA;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_error & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature_starboard & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature_port & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature_starboard & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature_port & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 9;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 10;
}
static void canzero_serialize_canzero_message_heartbeat(canzero_message_heartbeat* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x17B;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->node_id & (0xFF >> (8 - 4)));
}
static void canzero_serialize_canzero_message_get_req(canzero_message_get_req* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x13B;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->header.client_id << 13;
  ((uint32_t*)data)[0] |= msg->header.server_id << 21;
}
static void canzero_serialize_canzero_message_set_req(canzero_message_set_req* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x15B;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->header.sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->header.client_id << 16;
  ((uint32_t*)data)[0] |= msg->header.server_id << 24;
  ((uint32_t*)data)[1] = msg->data;
}
static void canzero_serialize_canzero_message_master_stream_mgu_control(canzero_message_master_stream_mgu_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x53;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->mgu_command & (0xFF >> (8 - 3)));
}
static void canzero_deserialize_canzero_message_get_resp(canzero_frame* frame, canzero_message_get_resp* msg) {
  uint8_t* data = frame->data;
  msg->header.sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->header.eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_set_resp(canzero_frame* frame, canzero_message_set_resp* msg) {
  uint8_t* data = frame->data;
  msg->header.od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.erno = (set_resp_erno)((((uint32_t*)data)[0] >> 29) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mgu1_stream_state(canzero_frame* frame, canzero_message_mgu1_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mgu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mgu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mgu1_stream_air_gaps(canzero_frame* frame, canzero_message_mgu1_stream_air_gaps* msg) {
  uint8_t* data = frame->data;
  msg->air_gap_starboard = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0;
  msg->air_gap_port = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + 0;
  msg->target_force_starboard = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + 0;
  msg->target_force_port = ((((uint32_t*)data)[1] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + 0;
}
static void canzero_deserialize_canzero_message_mgu1_stream_power_estimation(canzero_frame* frame, canzero_message_mgu1_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current_starboard = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->dc_current_port = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mgu1_stream_temperatures(canzero_frame* frame, canzero_message_mgu1_stream_temperatures* msg) {
  uint8_t* data = frame->data;
  msg->magnet_temperature_starboard = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1;
  msg->magnet_temperature_port = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1;
  msg->mcu_temperature = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1;
  msg->mosfet_temperature = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1;
}
static void canzero_deserialize_canzero_message_mgu1_stream_sdc_status(canzero_frame* frame, canzero_message_mgu1_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mgu1_stream_errors(canzero_frame* frame, canzero_message_mgu1_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_error = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature_starboard = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature_port = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature_starboard = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature_port = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_heartbeat(canzero_frame* frame, canzero_message_heartbeat* msg) {
  uint8_t* data = frame->data;
  msg->node_id = (node_id)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
}
static void canzero_deserialize_canzero_message_get_req(canzero_frame* frame, canzero_message_get_req* msg) {
  uint8_t* data = frame->data;
  msg->header.od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
}
static void canzero_deserialize_canzero_message_set_req(canzero_frame* frame, canzero_message_set_req* msg) {
  uint8_t* data = frame->data;
  msg->header.sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->header.eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_master_stream_mgu_control(canzero_frame* frame, canzero_message_master_stream_mgu_control* msg) {
  uint8_t* data = frame->data;
  msg->mgu_command = (mgu_command)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
}
void canzero_can0_poll();
void canzero_can1_poll();
uint32_t canzero_update_continue(uint32_t delta_time);
void canzero_init();
void canzero_set_state(mgu_state value);
void canzero_set_command(mgu_command value);
static inline void canzero_set_control_config(mgu_pid_values value){
  extern mgu_pid_values __oe_control_config;
  __oe_control_config = value;
}
void canzero_set_sdc_status(sdc_status value);
static inline void canzero_set_air_gap_starboard(float value){
  extern float __oe_air_gap_starboard;
  __oe_air_gap_starboard = value;
}
static inline void canzero_set_air_gap_port(float value){
  extern float __oe_air_gap_port;
  __oe_air_gap_port = value;
}
static inline void canzero_set_target_force_starboard(float value){
  extern float __oe_target_force_starboard;
  __oe_target_force_starboard = value;
}
static inline void canzero_set_target_force_port(float value){
  extern float __oe_target_force_port;
  __oe_target_force_port = value;
}
static inline void canzero_set_dc_current_starboard(float value){
  extern float __oe_dc_current_starboard;
  __oe_dc_current_starboard = value;
}
static inline void canzero_set_dc_current_port(float value){
  extern float __oe_dc_current_port;
  __oe_dc_current_port = value;
}
static inline void canzero_set_power_estimation(float value){
  extern float __oe_power_estimation;
  __oe_power_estimation = value;
}
static inline void canzero_set_magnet_temperature_starboard(float value){
  extern float __oe_magnet_temperature_starboard;
  __oe_magnet_temperature_starboard = value;
}
static inline void canzero_set_magnet_temperature_port(float value){
  extern float __oe_magnet_temperature_port;
  __oe_magnet_temperature_port = value;
}
static inline void canzero_set_mcu_temperature(float value){
  extern float __oe_mcu_temperature;
  __oe_mcu_temperature = value;
}
static inline void canzero_set_mosfet_temperature(float value){
  extern float __oe_mosfet_temperature;
  __oe_mosfet_temperature = value;
}
void canzero_set_error_45V_over_voltage(error_flag value);
void canzero_set_error_45V_under_voltage(error_flag value);
void canzero_set_error_control_error(error_flag value);
void canzero_set_error_magnet_over_temperature_starboard(error_flag value);
void canzero_set_error_magnet_over_temperature_port(error_flag value);
void canzero_set_error_mosfet_over_temperature(error_flag value);
void canzero_set_error_mcu_over_temperature(error_flag value);
void canzero_set_warn_magnet_over_temperature_starboard(error_flag value);
void canzero_set_warn_magnet_over_temperature_port(error_flag value);
void canzero_set_warn_mosfet_over_temperature(error_flag value);
void canzero_set_warn_mcu_over_temperature(error_flag value);
#endif