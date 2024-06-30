#ifndef CANZERO_H
#define CANZERO_H
#include <cinttypes>
#include <cstddef>
#define MAX_DYN_HEARTBEATS 10
typedef enum {
  node_id_gamepad = 0,
  node_id_mother_board = 1,
  node_id_motor_driver = 2,
  node_id_guidance_board_front = 3,
  node_id_guidance_board_back = 4,
  node_id_levitation_board1 = 5,
  node_id_levitation_board2 = 6,
  node_id_levitation_board3 = 7,
  node_id_input_board = 8,
  node_id_power_board12 = 9,
  node_id_power_board24 = 10,
  node_id_led_board = 11,
  node_id_count = 12,
} node_id;
typedef struct {
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
} get_req_header;
typedef struct {
  uint8_t m_sof;
  uint8_t m_eof;
  uint8_t m_toggle;
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
} set_req_header;
typedef enum {
  guidance_command_NONE = 0,
  guidance_command_ARM45 = 1,
  guidance_command_PRECHARGE = 2,
  guidance_command_START = 3,
  guidance_command_STOP = 4,
  guidance_command_DISARM45 = 5,
} guidance_command;
typedef enum {
  bool_t_FALSE = 0,
  bool_t_TRUE = 1,
} bool_t;
typedef struct {
  uint8_t m_sof;
  uint8_t m_eof;
  uint8_t m_toggle;
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
} get_resp_header;
typedef enum {
  set_resp_erno_Success = 0,
  set_resp_erno_Error = 1,
} set_resp_erno;
typedef struct {
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
  set_resp_erno m_erno;
} set_resp_header;
typedef enum {
  guidance_state_INIT = 0,
  guidance_state_IDLE = 1,
  guidance_state_ARMING45 = 2,
  guidance_state_PRECHARGE = 3,
  guidance_state_READY = 4,
  guidance_state_CONTROL = 5,
  guidance_state_DISARMING45 = 6,
} guidance_state;
typedef enum {
  sdc_status_OPEN = 0,
  sdc_status_CLOSED = 1,
} sdc_status;
typedef enum {
  error_flag_OK = 0,
  error_flag_ERROR = 1,
} error_flag;
typedef enum {
  error_level_OK = 0,
  error_level_INFO = 1,
  error_level_WARNING = 2,
  error_level_ERROR = 3,
} error_level;
typedef struct {
  uint16_t m_year;
  uint8_t m_month;
  uint8_t m_day;
  uint8_t m_hour;
  uint8_t m_min;
  uint8_t m_sec;
} date_time;
typedef struct {
  float m_info_thresh;
  float m_info_timeout;
  float m_warning_thresh;
  float m_warning_timeout;
  float m_error_thresh;
  float m_error_timeout;
  bool_t m_ignore_info;
  bool_t m_ignore_warning;
  bool_t m_ignore_error;
} error_level_config;
static const node_id CANZERO_NODE_ID = node_id_guidance_board_front;
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
static inline uint64_t canzero_get_config_hash() {
  extern uint64_t __oe_config_hash;
  return __oe_config_hash;
}
static inline date_time canzero_get_build_time() {
  extern date_time __oe_build_time;
  return __oe_build_time;
}
static inline guidance_state canzero_get_state() {
  extern guidance_state __oe_state;
  return __oe_state;
}
static inline sdc_status canzero_get_sdc_status() {
  extern sdc_status __oe_sdc_status;
  return __oe_sdc_status;
}
static inline error_flag canzero_get_assertion_fault() {
  extern error_flag __oe_assertion_fault;
  return __oe_assertion_fault;
}
static inline error_flag canzero_get_error_arming_failed() {
  extern error_flag __oe_error_arming_failed;
  return __oe_error_arming_failed;
}
static inline error_flag canzero_get_error_precharge_failed() {
  extern error_flag __oe_error_precharge_failed;
  return __oe_error_precharge_failed;
}
static inline error_flag canzero_get_error_heartbeat_miss() {
  extern error_flag __oe_error_heartbeat_miss;
  return __oe_error_heartbeat_miss;
}
static inline error_level canzero_get_error_level_vdc_voltage() {
  extern error_level __oe_error_level_vdc_voltage;
  return __oe_error_level_vdc_voltage;
}
static inline error_level canzero_get_error_level_magnet_current_left() {
  extern error_level __oe_error_level_magnet_current_left;
  return __oe_error_level_magnet_current_left;
}
static inline error_level canzero_get_error_level_magnet_current_right() {
  extern error_level __oe_error_level_magnet_current_right;
  return __oe_error_level_magnet_current_right;
}
static inline error_level canzero_get_error_level_input_current() {
  extern error_level __oe_error_level_input_current;
  return __oe_error_level_input_current;
}
static inline error_level canzero_get_error_level_magnet_temperature_left() {
  extern error_level __oe_error_level_magnet_temperature_left;
  return __oe_error_level_magnet_temperature_left;
}
static inline error_level canzero_get_error_level_magnet_temperature_right() {
  extern error_level __oe_error_level_magnet_temperature_right;
  return __oe_error_level_magnet_temperature_right;
}
static inline error_level canzero_get_error_level_mcu_temperature() {
  extern error_level __oe_error_level_mcu_temperature;
  return __oe_error_level_mcu_temperature;
}
static inline guidance_command canzero_get_command() {
  extern guidance_command __oe_command;
  return __oe_command;
}
static inline sdc_status canzero_get_precharge_status() {
  extern sdc_status __oe_precharge_status;
  return __oe_precharge_status;
}
static inline sdc_status canzero_get_feedthrough_status() {
  extern sdc_status __oe_feedthrough_status;
  return __oe_feedthrough_status;
}
static inline bool_t canzero_get_control_active() {
  extern bool_t __oe_control_active;
  return __oe_control_active;
}
static inline float canzero_get_outer_airgap_left() {
  extern float __oe_outer_airgap_left;
  return __oe_outer_airgap_left;
}
static inline float canzero_get_inner_airgap_left() {
  extern float __oe_inner_airgap_left;
  return __oe_inner_airgap_left;
}
static inline float canzero_get_outer_airgap_right() {
  extern float __oe_outer_airgap_right;
  return __oe_outer_airgap_right;
}
static inline float canzero_get_inner_airgap_right() {
  extern float __oe_inner_airgap_right;
  return __oe_inner_airgap_right;
}
static inline float canzero_get_vdc_voltage() {
  extern float __oe_vdc_voltage;
  return __oe_vdc_voltage;
}
static inline error_level_config canzero_get_error_level_config_vdc_voltage() {
  extern error_level_config __oe_error_level_config_vdc_voltage;
  return __oe_error_level_config_vdc_voltage;
}
static inline float canzero_get_current_left() {
  extern float __oe_current_left;
  return __oe_current_left;
}
static inline float canzero_get_current_right() {
  extern float __oe_current_right;
  return __oe_current_right;
}
static inline float canzero_get_input_current() {
  extern float __oe_input_current;
  return __oe_input_current;
}
static inline error_level_config canzero_get_error_level_config_magnet_current() {
  extern error_level_config __oe_error_level_config_magnet_current;
  return __oe_error_level_config_magnet_current;
}
static inline error_level_config canzero_get_error_level_config_input_current() {
  extern error_level_config __oe_error_level_config_input_current;
  return __oe_error_level_config_input_current;
}
static inline float canzero_get_magnet_temperature_left1() {
  extern float __oe_magnet_temperature_left1;
  return __oe_magnet_temperature_left1;
}
static inline float canzero_get_magnet_temperature_left2() {
  extern float __oe_magnet_temperature_left2;
  return __oe_magnet_temperature_left2;
}
static inline float canzero_get_magnet_temperature_left_max() {
  extern float __oe_magnet_temperature_left_max;
  return __oe_magnet_temperature_left_max;
}
static inline float canzero_get_magnet_temperature_right1() {
  extern float __oe_magnet_temperature_right1;
  return __oe_magnet_temperature_right1;
}
static inline float canzero_get_magnet_temperature_right2() {
  extern float __oe_magnet_temperature_right2;
  return __oe_magnet_temperature_right2;
}
static inline float canzero_get_magnet_temperature_right_max() {
  extern float __oe_magnet_temperature_right_max;
  return __oe_magnet_temperature_right_max;
}
static inline error_level_config canzero_get_error_level_config_magnet_temperature() {
  extern error_level_config __oe_error_level_config_magnet_temperature;
  return __oe_error_level_config_magnet_temperature;
}
static inline float canzero_get_mcu_temperature() {
  extern float __oe_mcu_temperature;
  return __oe_mcu_temperature;
}
static inline error_level_config canzero_get_error_level_config_mcu_temperature() {
  extern error_level_config __oe_error_level_config_mcu_temperature;
  return __oe_error_level_config_mcu_temperature;
}
static inline float canzero_get_loop_frequency() {
  extern float __oe_loop_frequency;
  return __oe_loop_frequency;
}
static inline float canzero_get_gamepad_lt2() {
  extern float __oe_gamepad_lt2;
  return __oe_gamepad_lt2;
}
static inline float canzero_get_gamepad_rt2() {
  extern float __oe_gamepad_rt2;
  return __oe_gamepad_rt2;
}
static inline float canzero_get_gamepad_lsb_x() {
  extern float __oe_gamepad_lsb_x;
  return __oe_gamepad_lsb_x;
}
static inline float canzero_get_gamepad_lsb_y() {
  extern float __oe_gamepad_lsb_y;
  return __oe_gamepad_lsb_y;
}
static inline float canzero_get_gamepad_rsb_x() {
  extern float __oe_gamepad_rsb_x;
  return __oe_gamepad_rsb_x;
}
static inline float canzero_get_gamepad_rsb_y() {
  extern float __oe_gamepad_rsb_y;
  return __oe_gamepad_rsb_y;
}
static inline bool_t canzero_get_gamepad_lt1_down() {
  extern bool_t __oe_gamepad_lt1_down;
  return __oe_gamepad_lt1_down;
}
static inline bool_t canzero_get_gamepad_rt1_down() {
  extern bool_t __oe_gamepad_rt1_down;
  return __oe_gamepad_rt1_down;
}
static inline bool_t canzero_get_gamepad_x_down() {
  extern bool_t __oe_gamepad_x_down;
  return __oe_gamepad_x_down;
}
static inline bool_t canzero_get_ignore_45v() {
  extern bool_t __oe_ignore_45v;
  return __oe_ignore_45v;
}
typedef struct {
  get_resp_header m_header;
  uint32_t m_data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0x11D;
typedef struct {
  set_resp_header m_header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0x13D;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
  guidance_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_guidance_board_front_stream_state;
static const uint32_t canzero_message_guidance_board_front_stream_state_id = 0xB3;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_magnet_current_left;
  error_level m_error_level_magnet_current_right;
  error_level m_error_level_input_current;
  error_level m_error_level_magnet_temperature_left;
  error_level m_error_level_magnet_temperature_right;
  error_level m_error_level_mcu_temperature;
} canzero_message_guidance_board_front_stream_errors;
static const uint32_t canzero_message_guidance_board_front_stream_errors_id = 0x93;
typedef struct {
  float m_loop_frequency;
} canzero_message_guidance_board_front_stream_debug;
static const uint32_t canzero_message_guidance_board_front_stream_debug_id = 0xFD;
typedef struct {
  float m_vdc_voltage;
  float m_current_left;
  float m_current_right;
  float m_input_current;
} canzero_message_guidance_board_front_stream_voltage_and_currents;
static const uint32_t canzero_message_guidance_board_front_stream_voltage_and_currents_id = 0x7C;
typedef struct {
  float m_outer_airgap_left;
  float m_inner_airgap_left;
  float m_outer_airgap_right;
  float m_inner_airgap_right;
} canzero_message_guidance_board_front_stream_airgaps;
static const uint32_t canzero_message_guidance_board_front_stream_airgaps_id = 0xDD;
typedef struct {
  float m_magnet_temperature_left1;
  float m_magnet_temperature_right1;
  float m_mcu_temperature;
} canzero_message_guidance_board_front_stream_temperatures;
static const uint32_t canzero_message_guidance_board_front_stream_temperatures_id = 0x5C;
typedef struct {
  uint8_t m_node_id;
  uint8_t m_unregister;
  uint8_t m_ticks_next;
} canzero_message_heartbeat_can0;
static const uint32_t canzero_message_heartbeat_can0_id = 0x14F;
typedef struct {
  uint8_t m_node_id;
  uint8_t m_unregister;
  uint8_t m_ticks_next;
} canzero_message_heartbeat_can1;
static const uint32_t canzero_message_heartbeat_can1_id = 0x14E;
typedef struct {
  get_req_header m_header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0x11E;
typedef struct {
  set_req_header m_header;
  uint32_t m_data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0x13E;
typedef struct {
  guidance_command m_guidance_command;
} canzero_message_mother_board_stream_guidance_command;
static const uint32_t canzero_message_mother_board_stream_guidance_command_id = 0x4C;
typedef struct {
  bool_t m_ignore_45v;
} canzero_message_mother_board_stream_debug_settings;
static const uint32_t canzero_message_mother_board_stream_debug_settings_id = 0x4D;
typedef struct {
  float m_lt2;
  float m_rt2;
  float m_lsb_x;
  float m_lsb_y;
  float m_rsb_x;
  float m_rsb_y;
  bool_t m_lt1_down;
  bool_t m_rt1_down;
  bool_t m_x_down;
  bool_t m_y_down;
  bool_t m_b_down;
  bool_t m_a_down;
  bool_t m_lsb_down;
  bool_t m_rsb_down;
} canzero_message_gamepad_stream_input;
static const uint32_t canzero_message_gamepad_stream_input_id = 0x5F;
void canzero_can0_poll();
void canzero_can1_poll();
uint32_t canzero_update_continue(uint32_t delta_time);
void canzero_init();
static inline void canzero_set_config_hash(uint64_t value){
  extern uint64_t __oe_config_hash;
  __oe_config_hash = value;
}

static inline void canzero_set_build_time(date_time value){
  extern date_time __oe_build_time;
  __oe_build_time = value;
}

void canzero_set_state(guidance_state value);

void canzero_set_sdc_status(sdc_status value);

void canzero_set_assertion_fault(error_flag value);

void canzero_set_error_arming_failed(error_flag value);

void canzero_set_error_precharge_failed(error_flag value);

void canzero_set_error_heartbeat_miss(error_flag value);

void canzero_set_error_level_vdc_voltage(error_level value);

void canzero_set_error_level_magnet_current_left(error_level value);

void canzero_set_error_level_magnet_current_right(error_level value);

void canzero_set_error_level_input_current(error_level value);

void canzero_set_error_level_magnet_temperature_left(error_level value);

void canzero_set_error_level_magnet_temperature_right(error_level value);

void canzero_set_error_level_mcu_temperature(error_level value);

void canzero_set_command(guidance_command value);

void canzero_set_precharge_status(sdc_status value);

void canzero_set_feedthrough_status(sdc_status value);

void canzero_set_control_active(bool_t value);

static inline void canzero_set_outer_airgap_left(float value){
  extern float __oe_outer_airgap_left;
  __oe_outer_airgap_left = value;
}

static inline void canzero_set_inner_airgap_left(float value){
  extern float __oe_inner_airgap_left;
  __oe_inner_airgap_left = value;
}

static inline void canzero_set_outer_airgap_right(float value){
  extern float __oe_outer_airgap_right;
  __oe_outer_airgap_right = value;
}

static inline void canzero_set_inner_airgap_right(float value){
  extern float __oe_inner_airgap_right;
  __oe_inner_airgap_right = value;
}

static inline void canzero_set_vdc_voltage(float value){
  extern float __oe_vdc_voltage;
  __oe_vdc_voltage = value;
}

static inline void canzero_set_error_level_config_vdc_voltage(error_level_config value){
  extern error_level_config __oe_error_level_config_vdc_voltage;
  __oe_error_level_config_vdc_voltage = value;
}

static inline void canzero_set_current_left(float value){
  extern float __oe_current_left;
  __oe_current_left = value;
}

static inline void canzero_set_current_right(float value){
  extern float __oe_current_right;
  __oe_current_right = value;
}

static inline void canzero_set_input_current(float value){
  extern float __oe_input_current;
  __oe_input_current = value;
}

static inline void canzero_set_error_level_config_magnet_current(error_level_config value){
  extern error_level_config __oe_error_level_config_magnet_current;
  __oe_error_level_config_magnet_current = value;
}

static inline void canzero_set_error_level_config_input_current(error_level_config value){
  extern error_level_config __oe_error_level_config_input_current;
  __oe_error_level_config_input_current = value;
}

static inline void canzero_set_magnet_temperature_left1(float value){
  extern float __oe_magnet_temperature_left1;
  __oe_magnet_temperature_left1 = value;
}

static inline void canzero_set_magnet_temperature_left2(float value){
  extern float __oe_magnet_temperature_left2;
  __oe_magnet_temperature_left2 = value;
}

static inline void canzero_set_magnet_temperature_left_max(float value){
  extern float __oe_magnet_temperature_left_max;
  __oe_magnet_temperature_left_max = value;
}

static inline void canzero_set_magnet_temperature_right1(float value){
  extern float __oe_magnet_temperature_right1;
  __oe_magnet_temperature_right1 = value;
}

static inline void canzero_set_magnet_temperature_right2(float value){
  extern float __oe_magnet_temperature_right2;
  __oe_magnet_temperature_right2 = value;
}

static inline void canzero_set_magnet_temperature_right_max(float value){
  extern float __oe_magnet_temperature_right_max;
  __oe_magnet_temperature_right_max = value;
}

static inline void canzero_set_error_level_config_magnet_temperature(error_level_config value){
  extern error_level_config __oe_error_level_config_magnet_temperature;
  __oe_error_level_config_magnet_temperature = value;
}

static inline void canzero_set_mcu_temperature(float value){
  extern float __oe_mcu_temperature;
  __oe_mcu_temperature = value;
}

static inline void canzero_set_error_level_config_mcu_temperature(error_level_config value){
  extern error_level_config __oe_error_level_config_mcu_temperature;
  __oe_error_level_config_mcu_temperature = value;
}

static inline void canzero_set_loop_frequency(float value){
  extern float __oe_loop_frequency;
  __oe_loop_frequency = value;
}

static inline void canzero_set_gamepad_lt2(float value){
  extern float __oe_gamepad_lt2;
  __oe_gamepad_lt2 = value;
}

static inline void canzero_set_gamepad_rt2(float value){
  extern float __oe_gamepad_rt2;
  __oe_gamepad_rt2 = value;
}

static inline void canzero_set_gamepad_lsb_x(float value){
  extern float __oe_gamepad_lsb_x;
  __oe_gamepad_lsb_x = value;
}

static inline void canzero_set_gamepad_lsb_y(float value){
  extern float __oe_gamepad_lsb_y;
  __oe_gamepad_lsb_y = value;
}

static inline void canzero_set_gamepad_rsb_x(float value){
  extern float __oe_gamepad_rsb_x;
  __oe_gamepad_rsb_x = value;
}

static inline void canzero_set_gamepad_rsb_y(float value){
  extern float __oe_gamepad_rsb_y;
  __oe_gamepad_rsb_y = value;
}

static inline void canzero_set_gamepad_lt1_down(bool_t value){
  extern bool_t __oe_gamepad_lt1_down;
  __oe_gamepad_lt1_down = value;
}

static inline void canzero_set_gamepad_rt1_down(bool_t value){
  extern bool_t __oe_gamepad_rt1_down;
  __oe_gamepad_rt1_down = value;
}

static inline void canzero_set_gamepad_x_down(bool_t value){
  extern bool_t __oe_gamepad_x_down;
  __oe_gamepad_x_down = value;
}

static inline void canzero_set_ignore_45v(bool_t value){
  extern bool_t __oe_ignore_45v;
  __oe_ignore_45v = value;
}

void canzero_send_config_hash();

void canzero_send_build_time();

void canzero_send_state();

void canzero_send_sdc_status();

void canzero_send_assertion_fault();

void canzero_send_error_arming_failed();

void canzero_send_error_precharge_failed();

void canzero_send_error_heartbeat_miss();

void canzero_send_error_level_vdc_voltage();

void canzero_send_error_level_magnet_current_left();

void canzero_send_error_level_magnet_current_right();

void canzero_send_error_level_input_current();

void canzero_send_error_level_magnet_temperature_left();

void canzero_send_error_level_magnet_temperature_right();

void canzero_send_error_level_mcu_temperature();

void canzero_send_command();

void canzero_send_precharge_status();

void canzero_send_feedthrough_status();

void canzero_send_control_active();

void canzero_send_outer_airgap_left();

void canzero_send_inner_airgap_left();

void canzero_send_outer_airgap_right();

void canzero_send_inner_airgap_right();

void canzero_send_vdc_voltage();

void canzero_send_error_level_config_vdc_voltage();

void canzero_send_current_left();

void canzero_send_current_right();

void canzero_send_input_current();

void canzero_send_error_level_config_magnet_current();

void canzero_send_error_level_config_input_current();

void canzero_send_magnet_temperature_left1();

void canzero_send_magnet_temperature_left2();

void canzero_send_magnet_temperature_left_max();

void canzero_send_magnet_temperature_right1();

void canzero_send_magnet_temperature_right2();

void canzero_send_magnet_temperature_right_max();

void canzero_send_error_level_config_magnet_temperature();

void canzero_send_mcu_temperature();

void canzero_send_error_level_config_mcu_temperature();

void canzero_send_loop_frequency();

void canzero_send_gamepad_lt2();

void canzero_send_gamepad_rt2();

void canzero_send_gamepad_lsb_x();

void canzero_send_gamepad_lsb_y();

void canzero_send_gamepad_rsb_x();

void canzero_send_gamepad_rsb_y();

void canzero_send_gamepad_lt1_down();

void canzero_send_gamepad_rt1_down();

void canzero_send_gamepad_x_down();

void canzero_send_ignore_45v();

#endif