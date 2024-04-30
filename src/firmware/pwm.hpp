#pragma once

#include "util/metrics.hpp"
#include "util/timestamp.hpp"
#include <cstdint>
#include <functional>
#include <imxrt.h>
#include <optional>

static constexpr bool ENABLE_PWM2_SM0 = true;
static constexpr bool ENABLE_PWM2_SM2 = true;
static constexpr bool ENABLE_PWM2_SM3 = true;
static constexpr bool ENABLE_PWM4_SM2 = true;
static constexpr bool ENABLE_PWM3_SM1 = true;
static constexpr bool ENABLE_PWM1_SM3 = true;

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
  std::optional<float> trig1 = std::nullopt;
  bool enable_trig1_interrupt = false;
  PwmControl control = PwmControl();
};

struct pwm {
public:
  static constexpr int TRIG0_SIGNAL_SOURCE = XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0;
  static constexpr int TRIG1_SIGNAL_SOURCE = XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG1;

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
  static PwmControl m_control;
  static bool m_outen;
  static bool m_trig0_inten;
  static bool m_trig1_inten;
  static std::optional<float> m_trig0;
  static std::optional<float> m_trig1;

  static volatile uint32_t m_pwm_cycles;
  static volatile uint32_t m_deadtime_cycles;
};
