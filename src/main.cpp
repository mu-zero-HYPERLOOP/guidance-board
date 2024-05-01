

#include "core_pins.h"
#include "firmware/adc_etc.hpp"
#include "firmware/pwm.hpp"
#include "firmware/xbar.hpp"
#include "imxrt.h"
#include "util/ema.hpp"
#include "util/timestamp.hpp"
#include "util/timimg.hpp"
#include <Arduino.h>

volatile IntervalTiming g_done0_timing;
volatile IntervalTiming g_done1_timing;

volatile Timing g_adc_timing;
ExponentialMovingAverage<Time> g_trig_time_filter(0.1, 1_s);
volatile Timing g_trig_timing;
ExponentialMovingAverage<Time> g_adc_time_filter(0.1, 1_s);

void pwm_trig0_isr() {
  g_adc_timing.start();
  g_trig_timing.start();
  // Not used right now
}


void pwm_trig1_isr() {
  g_trig_time_filter.push(static_cast<Time>(g_trig_timing.time()));
  // Not used right now
}


void adc_etc_done0_isr(AdcTrigRes res) {
  g_adc_time_filter.push(static_cast<Time>(g_adc_timing.time()));
  g_done0_timing.tick();
}


void adc_etc_done1_isr(AdcTrigRes res) {
  g_done1_timing.tick();
}




int main() {

  Serial.begin(9600);

  /* delay(2000); */
  Serial.printf("Hello, World\n");

  PwmBeginInfo pwmBeginInfo;
  pwmBeginInfo.enable_outputs = false;
  pwmBeginInfo.frequency = 20_kHz;
  pwmBeginInfo.deadtime = 1000_ns;
  pwmBeginInfo.enable_trig0_interrupt = true;
  pwmBeginInfo.enable_trig1_interrupt = true;
  pwmBeginInfo.trig0 = 0.0f;
  pwmBeginInfo.trig1 = 0.25f;
  pwm::begin(pwmBeginInfo);

  TrigChainInfo chains[3];
  chains[0].trig_num = TRIG0;
  int chain0_pins[] = {0,1,2}; //TODO use propery pinouts
  chains[0].read_pins = chain0_pins;
  chains[0].chain_length = 3;
  chains[0].chain_priority = 0;
  chains[0].software_trig = false;
  chains[0].trig_sync = true;
  chains[0].intr = DONE0;

  chains[1].trig_num = TRIG4;
  int chain4_pins[] = {0,1,2};
  chains[1].read_pins = chain4_pins;
  chains[1].chain_length = 3;
  chains[1].chain_priority = 0;
  chains[1].software_trig = false;
  chains[1].trig_sync = true;
  chains[1].intr = NONE;

  chains[2].trig_num = TRIG1;
  int chain1_pins[] = {0,1,2};
  chains[2].read_pins = chain1_pins;
  chains[2].chain_length = 3;
  chains[2].chain_priority = 0;
  chains[2].software_trig = false;
  chains[2].trig_sync = false;
  chains[2].intr = DONE1;


  AdcEtcBeginInfo adcBeginInfo;
  adcBeginInfo.adc1_avg = HwAvg::SAMPLE_8;
  adcBeginInfo.adc1_clock_div = AdcClockDivider::NO_DIV;
  adcBeginInfo.adc1_high_speed = true;
  adcBeginInfo.adc1_sample_time = PERIOD_25;
  adcBeginInfo.adc2_avg = adcBeginInfo.adc1_avg;
  adcBeginInfo.adc2_clock_div = adcBeginInfo.adc1_clock_div;
  adcBeginInfo.adc2_high_speed = adcBeginInfo.adc2_high_speed;
  adcBeginInfo.adc2_sample_time = adcBeginInfo.adc2_sample_time;
  adcBeginInfo.num_chains = sizeof(chains) / sizeof(TrigChainInfo);
  adcBeginInfo.chains = chains;
  AdcEtc::begin(adcBeginInfo);

  // Hook PWM trig signals to adc trig signals
  xbar::connect(pwm::TRIG0_SIGNAL_SOURCE, AdcEtc::TRIG0_SIGNAL_SINK);
  xbar::connect(pwm::TRIG1_SIGNAL_SOURCE, AdcEtc::TRIG1_SIGNAL_SINK);

  while (true) {
    Serial.printf("DONE0 Frequency = %fHz, DONE1 Frequency = %fHz\n", static_cast<float>(g_done0_timing.frequency()),
        static_cast<float>(g_done1_timing.frequency()));
    Serial.printf("ADC Convertion time = %fus\n", static_cast<float>(g_adc_time_filter.get()) * 1e6);
    Serial.printf("Time between triggers = %fus\n", static_cast<float>(g_trig_time_filter.get()) * 1e6);
    delay(50);
  }
}

