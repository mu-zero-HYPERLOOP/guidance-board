#pragma once

#include "util/metrics.h"
#include <cstdint>
#include <optional>
//                                                pin number   - guidance - motor
static constexpr bool ENABLE_PWM2_SM0 = false; // pins 4, 33   -          - U1
static constexpr bool ENABLE_PWM2_SM2 = true;  // pins 6 , 9   - LEFT_L   - V2
static constexpr bool ENABLE_PWM2_SM3 = false; // pins 36, 37  -          - U2
static constexpr bool ENABLE_PWM4_SM2 = true;  // pins 2 , 3   - LEFT_R   - W2
static constexpr bool ENABLE_PWM3_SM1 = true;  // pins 29, 28  - RIGHT_L  - W1
static constexpr bool ENABLE_PWM1_SM3 = true;  // pins 8 , 7   - RIGHT_R  - V1

struct PwmControl {
  float duty20 = 0.5f; // range [0,1]
  float duty22 = 0.5f;
  float duty23 = 0.5f;
  float duty42 = 0.5f;
  float duty31 = 0.5f;
  float duty13 = 0.5f;
};

struct PwmBeginInfo {
  Frequency frequency = 20_kHz;
  Time deadtime = 1_ns;
  bool enable_outputs = false;
  std::optional<float> trig0 = std::nullopt;
  bool enable_trig0_interrupt = false;
  bool enable_trig0 = false;
  std::optional<float> trig1 = std::nullopt;
  bool enable_trig1_interrupt = false;
  bool enable_trig1 = false;
  PwmControl control = PwmControl();
};

struct pwm {
public:
  static int TRIG0_SIGNAL_SOURCE; //= XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0;
  static int TRIG1_SIGNAL_SOURCE; //= XBARA1_IN_FLEXPWM4_PWM3_OUT_TRIG1;

  static void begin(const PwmBeginInfo &beginInfo = PwmBeginInfo());

  /**
   * trig0 = 0.0f will raise trig0 at the start of the pwm cycle
   * trig0 = 0.5f will raise trig0 at the end of pwm cycle
   * trig0 = std::nullopt will never raise trig0
   */
  static void trig0(const std::optional<float> &trig0);
  static const std::optional<float> &trig0() { return m_trig0; }
  /**
   * trig1 = 0.0f will raise trig1 at the start of the pwm cycle
   * trig1 = 0.5f will raise trig1 at the end of pwm cycle
   * trig1 = std::nullopt will never raise trig1
   */
  static void trig1(const std::optional<float> &trig1);
  static const std::optional<float> &trig1() { return m_trig1; }

  static void enable_trig0_interrupt();
  static void disable_trig0_interrupt();
  static void enable_trig1_interrupt();
  static void disable_trig1_interrupt();

  static void enable_trig0();
  static void disable_trig0();
  static void enable_trig1();
  static void disable_trig1();

  inline static bool trig0_is_enabled(){
    return m_enable_trig0;
  }

  inline static bool trig1_is_enabled(){
    return m_enable_trig1;
  }

  static void enable_output();
  static void disable_output();

  static void frequency(const Frequency &frequency);
  static inline const Frequency &frequency() { return m_frequency; }

  static void deadtime(const Time &deadtime);
  static inline const Time &deadtime() { return m_deadtime; }

  static void control(const PwmControl &control);

private:
  static void write_control();

  static Frequency m_frequency;
  static Time m_deadtime;
  static bool m_control_initalized;
  static PwmControl m_control;
  static bool m_outen;
  static bool m_trig0_inten;
  static bool m_trig1_inten;
  static std::optional<float> m_trig0;
  static std::optional<float> m_trig1;

  static volatile uint16_t m_pwm_cycles;
  static volatile uint16_t m_deadtime_cycles;

  static volatile bool m_enable_trig0;
  static volatile bool m_enable_trig1;
};
