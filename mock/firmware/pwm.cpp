#include "firmware/pwm.h"

__attribute__((weak)) void pwm_trig0_isr() {}

__attribute__((weak)) void pwm_trig1_isr() {}

PwmControl __pwm_control;

void pwm::enable_output() {
  if (m_outen)
    return;
  m_outen = true;
}

void pwm::disable_output() {
  if (!m_outen)
    return;
  __pwm_control = PwmControl();
  m_outen = false;
}

void pwm::enable_trig0() {
  if (m_enable_trig0){
    return;
  }
  m_enable_trig0 = true;
}
void pwm::disable_trig0() {
  if (!m_enable_trig0){
    return;
  }
  m_enable_trig0 = false;
}
void pwm::enable_trig1() {
  if (m_enable_trig1){
    return;
  }
  m_enable_trig1 = true;
}
void pwm::disable_trig1() {
  if (!m_enable_trig1){
    return;
  }
  m_enable_trig1 = false;
}

void pwm::trig0(const std::optional<float> &trig0) {
  if (m_trig0 == trig0) {
    return;
  }
  m_trig0 = trig0;
  m_enable_trig0 = trig0.has_value();
}

void pwm::trig1(const std::optional<float> &trig1) {
  if (m_trig1 == trig1) {
    return;
  }
  m_trig1 = trig1;
  m_enable_trig1 = m_trig1.has_value();
}

void pwm::enable_trig0_interrupt() {
  if (m_trig0_inten) {
    return;
  }
  m_trig0_inten = true;
}

void pwm::disable_trig0_interrupt() {
  if (!m_trig0_inten) {
    return;
  }
  m_trig0_inten = false;
}

void pwm::enable_trig1_interrupt() {
  if (m_trig1_inten) {
    return;
  }
  m_trig1_inten = true;
}

void pwm::disable_trig1_interrupt() {
  if (!m_trig1_inten) {
    return;
  }
  m_trig1_inten = false;
}



void pwm::frequency(const Frequency &frequency) {
  m_frequency = frequency;
}

void pwm::write_control() {
}

void pwm::deadtime(const Time &deadtime) {
}


void pwm::control(const PwmControl &control) {
  m_control = control;
  __pwm_control = control;
}

void pwm::begin(const PwmBeginInfo &beginInfo) {
  m_frequency = beginInfo.frequency;
  m_deadtime = beginInfo.deadtime;
  m_outen = beginInfo.enable_outputs;
  m_trig0 = beginInfo.trig0;
  m_enable_trig0 = beginInfo.trig0.has_value();
  m_trig0_inten = beginInfo.enable_trig0_interrupt;
  m_trig1 = beginInfo.trig1;
  m_enable_trig1 = beginInfo.trig1.has_value();
  m_trig1_inten = beginInfo.enable_trig1_interrupt;
  m_control = beginInfo.control;
  __pwm_control = m_control;
}

Frequency pwm::m_frequency;
Time pwm::m_deadtime;
PwmControl pwm::m_control;
bool pwm::m_outen;
bool pwm::m_trig0_inten;
bool pwm::m_trig1_inten;
std::optional<float> pwm::m_trig0;
std::optional<float> pwm::m_trig1;
volatile bool pwm::m_enable_trig0;
volatile bool pwm::m_enable_trig1;
volatile uint16_t pwm::m_pwm_cycles;
volatile uint16_t pwm::m_deadtime_cycles;

int pwm::TRIG0_SIGNAL_SOURCE = 0;
int pwm::TRIG1_SIGNAL_SOURCE = 1;
