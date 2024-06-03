#include "pwm.h"

#include "core_pins.h"
#include "xbar.h"
#include <Arduino.h>
#include <imxrt.h>
#include <tuple>
#include <algorithm>


__attribute__((weak)) void pwm_trig0_isr() {

}
void pwm_isr_trig0_redirect() {
  FLEXPWM4_SM0STS |= FLEXPWM_SMSTS_CMPF(4); // Clear interrupt flag
  pwm_trig0_isr();
}

__attribute__((weak)) void pwm_trig1_isr() {

}
void pwm_isr_trig1_redirect() {
  FLEXPWM4_SM2STS |= FLEXPWM_SMSTS_CMPF(8); // Clear interrupt flag
  pwm_trig1_isr();
}


constexpr bool ENABLE_PWM1 = ENABLE_PWM1_SM3;
constexpr bool ENABLE_PWM2 =
    ENABLE_PWM2_SM0 || ENABLE_PWM2_SM2 || ENABLE_PWM2_SM3;
constexpr bool ENABLE_PWM3 = ENABLE_PWM3_SM1;
constexpr bool ENABLE_PWM4 = ENABLE_PWM4_SM2;

namespace pwm_reg {
// see https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf at p3135 for register
// description

static inline void clear_load_okay() {
  // Clear Load Okay -> no reload of PWM settings.
  if constexpr (ENABLE_PWM1) {
    FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  }
  if constexpr (ENABLE_PWM2) {
    FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  }
  if constexpr (ENABLE_PWM3) {
    FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  }
  if constexpr (ENABLE_PWM4) {
    FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);
  }
}
static inline void smxctrl_write_cycle_with_prescalar(bool fullcycle,
                                                      int prescalar) {
  // SMxCTRL:
  //  - FULL : Full Cycle Reload enabled.
  //  - PRSC : Prescalar (3bit) : PWM Clock Frequency is f = f_clk / (2 <<
  //  Prescalar - 1).
  if constexpr (ENABLE_PWM1_SM3) {
    uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
    if (fullcycle) {
      ctrl |= FLEXPWM_SMCTRL_FULL;
    } else {
      ctrl |= FLEXPWM_SMCTRL_HALF;
    }
    FLEXPWM1_SM3CTRL = ctrl;
  }
  if constexpr (ENABLE_PWM2_SM0) {
    uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
    if (fullcycle) {
      ctrl |= FLEXPWM_SMCTRL_FULL;
    } else {
      ctrl |= FLEXPWM_SMCTRL_HALF;
    }
    FLEXPWM2_SM0CTRL = ctrl;
  }
  if constexpr (ENABLE_PWM2_SM2) {
    uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
    if (fullcycle) {
      ctrl |= FLEXPWM_SMCTRL_FULL;
    } else {
      ctrl |= FLEXPWM_SMCTRL_HALF;
    }
    FLEXPWM2_SM2CTRL = ctrl;
  }
  if constexpr (ENABLE_PWM2_SM3) {
    uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
    if (fullcycle) {
      ctrl |= FLEXPWM_SMCTRL_FULL;
    } else {
      ctrl |= FLEXPWM_SMCTRL_HALF;
    }
    FLEXPWM2_SM3CTRL = ctrl;
  }
  if constexpr (ENABLE_PWM3_SM1) {
    uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
    if (fullcycle) {
      ctrl |= FLEXPWM_SMCTRL_FULL;
    } else {
      ctrl |= FLEXPWM_SMCTRL_HALF;
    }
    FLEXPWM3_SM1CTRL = ctrl;
  }
  if constexpr (ENABLE_PWM4_SM2) {
    uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
    if (fullcycle) {
      ctrl |= FLEXPWM_SMCTRL_FULL;
    } else {
      ctrl |= FLEXPWM_SMCTRL_HALF;
    }
    FLEXPWM4_SM2CTRL = ctrl;
  }
  uint16_t ctrl = FLEXPWM_SMCTRL_PRSC(prescalar);
  if (fullcycle) {
    ctrl |= FLEXPWM_SMCTRL_FULL;
  } else {
    ctrl |= FLEXPWM_SMCTRL_HALF;
  }
  FLEXPWM4_SM0CTRL = ctrl;
}

static inline void set_load_okay() {
  if constexpr (ENABLE_PWM1) {
    FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  }
  if constexpr (ENABLE_PWM2) {
    FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  }
  if constexpr (ENABLE_PWM3) {
    FLEXPWM3_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  }
  if constexpr (ENABLE_PWM4) {
    FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);
  }
}

static inline void set_output_enable(bool enable_outputs) {
  if constexpr (ENABLE_PWM1) {
    if (enable_outputs) {
      FLEXPWM1_OUTEN =
          FLEXPWM_OUTEN_PWMB_EN(0b1000) | FLEXPWM_OUTEN_PWMA_EN(0b1000);
    } else {
      FLEXPWM1_OUTEN = 0x0;
    }
  }
  if constexpr (ENABLE_PWM2) {
    if (enable_outputs) {
      uint16_t outen = 0; //<- should be consteval on -O2
      if (ENABLE_PWM2_SM0) {
        outen |= FLEXPWM_OUTEN_PWMB_EN(0b0001) | FLEXPWM_OUTEN_PWMA_EN(0b0001);
      }
      if (ENABLE_PWM2_SM2) {
        outen |= FLEXPWM_OUTEN_PWMB_EN(0b0100) | FLEXPWM_OUTEN_PWMA_EN(0b0100);
      }
      if (ENABLE_PWM2_SM3) {
        outen |= FLEXPWM_OUTEN_PWMB_EN(0b1000) | FLEXPWM_OUTEN_PWMA_EN(0b1000);
      }
      FLEXPWM2_OUTEN = outen;
    } else {
      FLEXPWM2_OUTEN = 0x0;
    }
  }
  if constexpr (ENABLE_PWM3) {
    if (enable_outputs) {
      FLEXPWM3_OUTEN =
          FLEXPWM_OUTEN_PWMB_EN(0b0010) | FLEXPWM_OUTEN_PWMA_EN(0b0010);
    } else {
      FLEXPWM3_OUTEN = 0;
    }
  }
  if constexpr (ENABLE_PWM4) {
    if (enable_outputs) {
      FLEXPWM4_OUTEN =
          FLEXPWM_OUTEN_PWMB_EN(0b0100) | FLEXPWM_OUTEN_PWMA_EN(0b0100);
    } else {
      FLEXPWM4_OUTEN = 0;
    }
  }
}

static inline void sm0_interrupt_enable(bool trig0, bool trig1) {
  if (trig0) {
    FLEXPWM4_SM0INTEN |= FLEXPWM_SMINTEN_CMPIE(0b010000);
    NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_0);
  }else {
    FLEXPWM4_SM0INTEN &= ~FLEXPWM_SMINTEN_CMPIE(0b010000);
    NVIC_DISABLE_IRQ(IRQ_FLEXPWM4_0);
  }
  if (trig1) {
    FLEXPWM4_SM2INTEN |= FLEXPWM_SMINTEN_CMPIE(0b100000);
    NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_2);
  }else {
    FLEXPWM4_SM2INTEN &= ~FLEXPWM_SMINTEN_CMPIE(0b100000);
    NVIC_DISABLE_IRQ(IRQ_FLEXPWM4_2);
  }
}

static inline void sm0tctrl_enable_output_triggers(bool trig0, bool trig1) {
  if (trig0) {
    // Raises TRIG0, when SM0_CNT == SM0_VAL4
    FLEXPWM4_SM0TCTRL |= FLEXPWM_SMTCTRL_OUT_TRIG_EN(0b010000);
  }else {
    FLEXPWM4_SM0TCTRL &= ~FLEXPWM_SMTCTRL_OUT_TRIG_EN(0b010000);
  }
  if (trig1) {
    // Raises TRIG1, when SM0_CNT == SM0_VAL5
    FLEXPWM4_SM2TCTRL |= FLEXPWM_SMTCTRL_OUT_TRIG_EN(0b100000);
  }else {
    FLEXPWM4_SM2TCTRL &= ~FLEXPWM_SMTCTRL_OUT_TRIG_EN(0b100000);
  }
}

static inline void smxctrl2_synchronize_pwms() {
  // CTRL2:
  //  -CLK_SEC : Selects the clock source. 0 selects the IPBus clock source.
  //  -INIT_SEL : Initalization Control Select :
  //        0b00 selects Local sync (PWM_X) cases initalization
  //        0b10 selects Master sync from SM0 causes initalization.
  //        0b11 selects EXT_SYC causes initalization.
  //  -RELOAD_SEL : The master RELOAD from SM0 is used.
  // Why do different submodules have different settings here?
  // -> different sources for syncronization
  // PWM4_SM0 is the source for sync

  // routing the EXT_SYNC over XBAR introduces a small delay, of ~28ns
  // different to SciMo, now every submodule is reset by EXT_SYNC, which
  // originates (for all SM) from PWM4_SM0 so every submodule is triggered at
  // the same time
  if constexpr (ENABLE_PWM1_SM3) {
    FLEXPWM1_SM3CTRL2 =
        FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  }
  if constexpr (ENABLE_PWM2_SM0) {
    FLEXPWM2_SM0CTRL2 =
        FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  }
  if constexpr (ENABLE_PWM2_SM2) {
    FLEXPWM2_SM2CTRL2 =
        FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  }
  if constexpr (ENABLE_PWM2_SM3) {
    FLEXPWM2_SM3CTRL2 =
        FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  }
  if constexpr (ENABLE_PWM3_SM1) {
    FLEXPWM3_SM1CTRL2 =
        FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  }
  if constexpr (ENABLE_PWM4_SM2) {
    FLEXPWM4_SM2CTRL2 =
        FLEXPWM_SMCTRL2_CLK_SEL(0) | FLEXPWM_SMCTRL2_INIT_SEL(0b11);
  }
  FLEXPWM4_SM0CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0);
}

static inline void sm0_set_trig0_value(int16_t trig0_cnt) {
  FLEXPWM4_SM0VAL4 = trig0_cnt;
}

static inline void sm0_set_trig1_value(int16_t trig1_cnt) {
  FLEXPWM4_SM2VAL5 = trig1_cnt;
}

static inline void pwm4_sm2_set_duty_cycles(uint16_t cycles) {
  FLEXPWM4_SM2VAL2 = -cycles;
  FLEXPWM4_SM2VAL3 = cycles;
}

static inline void pwm3_sm1_set_duty_cycles(uint16_t cycles) {
  FLEXPWM3_SM1VAL2 = -cycles;
  FLEXPWM3_SM1VAL3 = cycles;
}

static inline void pwm1_sm3_set_duty_cycles(uint16_t cycles) {
  FLEXPWM1_SM3VAL2 = -cycles;
  FLEXPWM1_SM3VAL3 = cycles;
}

static inline void pwm2_sm0_set_duty_cycles(uint16_t cycles) {
  FLEXPWM2_SM0VAL2 = -cycles;
  FLEXPWM2_SM0VAL3 = cycles;
}

static inline void pwm2_sm2_set_duty_cycles(uint16_t cycles) {
  FLEXPWM2_SM2VAL2 = -cycles;
  FLEXPWM2_SM2VAL3 = cycles;
}

static inline void pwm2_sm3_set_duty_cycles(uint16_t cycles) {
  FLEXPWM2_SM3VAL2 = -cycles;
  FLEXPWM2_SM3VAL3 = cycles;
}

static inline void smx_center_aligned_max_value(uint16_t cycles) {
  // Initalize PWM for a symetrical pulse around 0x0.
  //  - INIT is reload value (or start value)
  //  - VAL0 is the mid point
  //  - VAL1 is the max count before counter reset.
  // PWM 1 SM 3
  if constexpr (ENABLE_PWM1_SM3) {
    FLEXPWM1_SM3INIT = -cycles;
    FLEXPWM1_SM3VAL0 = 0;
    FLEXPWM1_SM3VAL1 = cycles;
  }
  // PWM 2 SM 0
  if constexpr (ENABLE_PWM2_SM0) {
    FLEXPWM2_SM0INIT = -cycles;
    FLEXPWM2_SM0VAL0 = 0;
    FLEXPWM2_SM0VAL1 = cycles;
  }
  // PWM 2 SM 2
  if constexpr (ENABLE_PWM2_SM2) {
    FLEXPWM2_SM2INIT = -cycles;
    FLEXPWM2_SM2VAL0 = 0;
    FLEXPWM2_SM2VAL1 = cycles;
  }
  // PWM 2 SM 3
  if constexpr (ENABLE_PWM2_SM3) {
    FLEXPWM2_SM3INIT = -cycles;
    FLEXPWM2_SM3VAL0 = 0;
    FLEXPWM2_SM3VAL1 = cycles;
  }
  // PWM 3 SM 1
  if constexpr (ENABLE_PWM3_SM1) {
    FLEXPWM3_SM1INIT = -cycles;
    FLEXPWM3_SM1VAL0 = 0;
    FLEXPWM3_SM1VAL1 = cycles;
  }
  // PWM 4 SM 2
  if constexpr (ENABLE_PWM4_SM2) {
    FLEXPWM4_SM2INIT = -cycles;
    FLEXPWM4_SM2VAL0 = 0;
    FLEXPWM4_SM2VAL1 = cycles;
  }

  // PWM 4 SM 0
  FLEXPWM4_SM0INIT = -cycles;
  FLEXPWM4_SM0VAL1 = cycles;
}

static inline void dtcnt_write_deadtime(const uint16_t cycles) {
  if constexpr (ENABLE_PWM1_SM3) {
    FLEXPWM1_SM3DTCNT0 = cycles;
    FLEXPWM1_SM3DTCNT1 = cycles;
  }
  // PWM 2 SM 0
  if constexpr (ENABLE_PWM2_SM0) {
    FLEXPWM2_SM0DTCNT0 = cycles;
    FLEXPWM2_SM0DTCNT1 = cycles;
  }
  // PWM 2 SM 2
  if constexpr (ENABLE_PWM2_SM2) {
    FLEXPWM2_SM2DTCNT0 = cycles;
    FLEXPWM2_SM2DTCNT1 = cycles;
  }
  // PWM w SM 3
  if constexpr (ENABLE_PWM2_SM3) {
    FLEXPWM2_SM3DTCNT0 = cycles;
    FLEXPWM2_SM3DTCNT1 = cycles;
  }
  // PWM 3 SM 1
  if constexpr (ENABLE_PWM3_SM1) {
    FLEXPWM3_SM1DTCNT0 = cycles;
    FLEXPWM3_SM1DTCNT1 = cycles;
  }
  // PWM 4 SM 2
  if constexpr (ENABLE_PWM4_SM2) {
    FLEXPWM4_SM2DTCNT0 = cycles;
    FLEXPWM4_SM2DTCNT1 = cycles;
  }
}

} // namespace pwm_reg

namespace conv {
static inline std::tuple<uint16_t, uint16_t>
frequency_to_cycles(const Frequency &frequency) {
  float f2 = 2.0f * static_cast<float>(frequency);
  uint16_t cycles =
      static_cast<uint32_t>(static_cast<float>(F_BUS_ACTUAL) / f2 + 0.5f);

  uint16_t prescalar = 0;
  while (cycles > 32767 && prescalar < 7) {
    cycles /= 2.0f;
    prescalar += 1;
  }
  if (cycles > 32767) {
    cycles = 32767;
  } else if (cycles < 2) {
    cycles = 2; // minimale Cycles --> 10nS oder so
  }
  return {cycles, prescalar};
}

static inline uint16_t deadtime_to_cycles(const Time &deadtime) {
  float deadtime_ns = static_cast<float>(deadtime) * 1e9f;
  return static_cast<uint16_t>(static_cast<float>(F_BUS_ACTUAL) * deadtime_ns *
                               1e-9f);
}

static inline int16_t trig_to_cnt(float trig, uint16_t cycles) {
  // cycles refers to the max value of the center aligned counter so half the
  // total amount of pwm cycles
  uint16_t edge_algined_cycles = cycles * 2;
  int16_t cnt = static_cast<uint16_t>(edge_algined_cycles * trig) - cycles;
  return cnt;
}
static inline uint16_t duty_to_cmp(float duty, uint16_t cycles) {
  duty = std::clamp(duty, 0.0f, 1.0f);
  return std::min(static_cast<uint32_t>(cycles), static_cast<uint32_t>(duty * cycles));
}
} // namespace conv

void pwm::enable_output() {
  if (m_outen)
    return;
  m_outen = true;
  pwm_reg::clear_load_okay();
  pwm_reg::set_output_enable(m_outen);
  pwm_reg::set_load_okay();
}

void pwm::disable_output() {
  if (!m_outen)
    return;
  m_outen = false;
  pwm_reg::clear_load_okay();
  pwm_reg::set_output_enable(m_outen);
  pwm_reg::set_load_okay();
}

void pwm::trig0(const std::optional<float> &trig0) {
  if (m_trig0 == trig0) {
    return;
  }
  m_trig0 = trig0;
  pwm_reg::clear_load_okay();
  pwm_reg::sm0tctrl_enable_output_triggers(m_trig0.has_value(),
                                           m_trig1.has_value());
  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);
  if (m_trig0.has_value()) {
    pwm_reg::sm0_set_trig0_value(
        conv::trig_to_cnt(m_trig0.value(), m_pwm_cycles));
  }

  if (m_trig1.has_value()) {
    pwm_reg::sm0_set_trig1_value(
        conv::trig_to_cnt(m_trig1.value(), m_pwm_cycles));
  }
  pwm_reg::set_load_okay();
}

void pwm::trig1(const std::optional<float> &trig1) {
  if (m_trig1 == trig1) {
    return;
  }
  m_trig1 = trig1;

  pwm_reg::clear_load_okay();
  pwm_reg::sm0tctrl_enable_output_triggers(m_trig0.has_value(),
                                           m_trig1.has_value());
  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);
  if (m_trig0.has_value()) {
    pwm_reg::sm0_set_trig0_value(
        conv::trig_to_cnt(m_trig0.value(), m_pwm_cycles));
  }

  if (m_trig1.has_value()) {
    pwm_reg::sm0_set_trig1_value(
        conv::trig_to_cnt(m_trig1.value(), m_pwm_cycles));
  }
  pwm_reg::set_load_okay();
}

void pwm::enable_trig0_interrupt() {
  if (m_trig0_inten) {
    return;
  }
  m_trig0_inten = true;
  pwm_reg::clear_load_okay();
  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);
  pwm_reg::set_load_okay();
}

void pwm::disable_trig0_interrupt() {
  if (!m_trig0_inten) {
    return;
  }
  m_trig0_inten = false;
  pwm_reg::clear_load_okay();
  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);
  pwm_reg::set_load_okay();
}

void pwm::enable_trig1_interrupt() {
  if (m_trig1_inten) {
    return;
  }
  m_trig1_inten = true;
  pwm_reg::clear_load_okay();
  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);
  pwm_reg::set_load_okay();
}

void pwm::disable_trig1_interrupt() {
  if (!m_trig1_inten) {
    return;
  }
  m_trig1_inten = false;
  pwm_reg::clear_load_okay();
  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);
  pwm_reg::set_load_okay();
}

void pwm::frequency(const Frequency &frequency) {
  const auto &[pwm_cycles, prescalar] =
      conv::frequency_to_cycles(m_frequency);
  m_pwm_cycles = pwm_cycles;
  pwm_reg::clear_load_okay();
  pwm_reg::smxctrl_write_cycle_with_prescalar(false, prescalar);
  pwm_reg::smx_center_aligned_max_value(m_pwm_cycles);
  if (m_trig0.has_value()) {
    pwm_reg::sm0_set_trig0_value(
        conv::trig_to_cnt(m_trig0.value(), m_pwm_cycles));
  }
  if (m_trig1.has_value()) {
    pwm_reg::sm0_set_trig1_value(
        conv::trig_to_cnt(m_trig1.value(), m_pwm_cycles));
  }
  write_control();
  pwm_reg::set_load_okay();
}

void pwm::write_control() {
  // Serial.printf("control = %f %u %u\n", m_control.duty42, m_pwm_cycles, 
      // conv::duty_to_cmp(m_control.duty42, m_pwm_cycles));
  if constexpr (ENABLE_PWM4_SM2) {
    pwm_reg::pwm4_sm2_set_duty_cycles(
        conv::duty_to_cmp(m_control.duty42, m_pwm_cycles));
  }
  if constexpr (ENABLE_PWM3_SM1) {
    pwm_reg::pwm3_sm1_set_duty_cycles(
        conv::duty_to_cmp(m_control.duty31, m_pwm_cycles));
  }
  if constexpr (ENABLE_PWM1_SM3) {
    pwm_reg::pwm1_sm3_set_duty_cycles(
        conv::duty_to_cmp(m_control.duty13, m_pwm_cycles));
  }
  if constexpr (ENABLE_PWM2_SM0) {
    pwm_reg::pwm2_sm0_set_duty_cycles(
        conv::duty_to_cmp(m_control.duty20, m_pwm_cycles));
  }
  if constexpr (ENABLE_PWM2_SM2) {
    pwm_reg::pwm2_sm2_set_duty_cycles(
        conv::duty_to_cmp(m_control.duty22, m_pwm_cycles));
  }
  if constexpr (ENABLE_PWM2_SM3) {
    pwm_reg::pwm2_sm3_set_duty_cycles(
        conv::duty_to_cmp(m_control.duty23, m_pwm_cycles));
  }
}

void pwm::deadtime(const Time &deadtime) {
  pwm_reg::clear_load_okay();
  m_deadtime_cycles = conv::deadtime_to_cycles(deadtime);
  pwm_reg::dtcnt_write_deadtime(m_deadtime_cycles);
  pwm_reg::set_load_okay();
}

void pwm::control(const PwmControl &control) {
  m_control = control;
  pwm_reg::clear_load_okay();
  write_control();
  pwm_reg::set_load_okay();
}

void pwm::begin(const PwmBeginInfo &beginInfo) {
  xbar::begin(); //<- only requires that xbar is initalized
  if constexpr (ENABLE_PWM1_SM3) {
    xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0,
                  XBARA1_OUT_FLEXPWM1_PWM3_EXT_SYNC);
  }
  if constexpr (ENABLE_PWM2_SM0) {
    xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0,
                  XBARA1_OUT_FLEXPWM2_PWM0_EXT_SYNC);
  }
  if constexpr (ENABLE_PWM4_SM2) {
    xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0,
                  XBARA1_OUT_FLEXPWM2_PWM2_EXT_SYNC);
  }
  if constexpr (ENABLE_PWM2_SM3) {
    xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0,
                  XBARA1_OUT_FLEXPWM2_PWM3_EXT_SYNC);
  }
  if constexpr (ENABLE_PWM3) {
    xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0,
                  XBARA1_OUT_FLEXPWM3_EXT_SYNC1);
  }
  if constexpr (ENABLE_PWM4) {
    xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0,
                  XBARA1_OUT_FLEXPWM4_EXT_SYNC2);
  }

  m_frequency = beginInfo.frequency;
  m_deadtime = beginInfo.deadtime;
  m_outen = beginInfo.enable_outputs;
  m_trig0 = beginInfo.trig0;
  m_trig0_inten = beginInfo.enable_trig0_interrupt;
  m_trig1 = beginInfo.trig1;
  m_trig1_inten = beginInfo.enable_trig1_interrupt;
  m_control = beginInfo.control;
  m_enable_trig0 = false;
  m_enable_trig1 = false;
  
  attachInterruptVector(IRQ_FLEXPWM4_0, pwm_isr_trig0_redirect);
  attachInterruptVector(IRQ_FLEXPWM4_2, pwm_isr_trig1_redirect);

  const auto &[pwm_cycles, prescalar] =
      conv::frequency_to_cycles(m_frequency);
  m_pwm_cycles = pwm_cycles;

  m_deadtime_cycles = conv::deadtime_to_cycles(m_deadtime);


  pwm_reg::clear_load_okay();

  pwm_reg::smxctrl_write_cycle_with_prescalar(true, prescalar);

  pwm_reg::smxctrl2_synchronize_pwms();

  pwm_reg::set_output_enable(m_outen);

  pwm_reg::sm0tctrl_enable_output_triggers(m_trig0.has_value(),
                                           m_trig1.has_value());

  pwm_reg::sm0_interrupt_enable(m_trig0.has_value() && m_trig0_inten,
                                m_trig1.has_value() && m_trig1_inten);

  pwm_reg::dtcnt_write_deadtime(m_deadtime_cycles);

  pwm_reg::smx_center_aligned_max_value(m_pwm_cycles);

  if (m_trig0.has_value()) {
    pwm_reg::sm0_set_trig0_value(
        conv::trig_to_cnt(m_trig0.value(), m_pwm_cycles));
  }

  if (m_trig1.has_value()) {
    pwm_reg::sm0_set_trig1_value(
        conv::trig_to_cnt(m_trig1.value(), m_pwm_cycles));
  }
  

  write_control();


  // Update pin configuration
  // Selects which module is connected to the pins, in this case it is the
  // flexpwm module the assigned values are from the datasheet chapter 11.6.xx
  // and dont follow an obvious rule i think

  if constexpr (ENABLE_PWM4_SM2) {
    *(portConfigRegister(2)) = 1;
    *(portConfigRegister(3)) = 1;
  }

  if constexpr (ENABLE_PWM3_SM1) {
    *(portConfigRegister(28)) = 1;
    *(portConfigRegister(29)) = 1;
  }

  if constexpr (ENABLE_PWM1_SM3) {
    *(portConfigRegister(7)) = 6;
    *(portConfigRegister(8)) = 6;
  }

  if constexpr (ENABLE_PWM2_SM0) {
    *(portConfigRegister(4)) = 1;
    *(portConfigRegister(33)) = 1;
  }

  if constexpr (ENABLE_PWM2_SM2) {
    *(portConfigRegister(6)) = 2;
    *(portConfigRegister(9)) = 2;
  }

  if constexpr (ENABLE_PWM2_SM3) {
    *(portConfigRegister(36)) = 6;
    *(portConfigRegister(37)) = 6;
  }

  // for some reason the PWM submodules in the XBAR inputs are labeled 1-4
  // instead of 0-3 like everywhere else so FLEXPWM4_PWM1 is submodule 0 of the
  // flexpwm 4 module

  // Load Okay -> reload setting again
  pwm_reg::set_load_okay();
}

Frequency pwm::m_frequency;
Time pwm::m_deadtime;
PwmControl pwm::m_control;
bool pwm::m_outen;
bool pwm::m_trig0_inten;
bool pwm::m_trig1_inten;
std::optional<float> pwm::m_trig0;
std::optional<float> pwm::m_trig1;
volatile uint16_t pwm::m_pwm_cycles;
volatile uint16_t pwm::m_deadtime_cycles;
volatile bool pwm::m_enable_trig0;
volatile bool pwm::m_enable_trig1;

int pwm::TRIG0_SIGNAL_SOURCE = XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0;
int pwm::TRIG1_SIGNAL_SOURCE = XBARA1_IN_FLEXPWM4_PWM3_OUT_TRIG1;
