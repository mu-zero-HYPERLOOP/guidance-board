

#include "firmware/pwm.hpp"
#include <usb_serial.h>


int main() {
  PwmBeginInfo pwmBeginInfo;
  pwmBeginInfo.enable_outputs = true;
  pwmBeginInfo.frequency = 20_kHz;
  pwmBeginInfo.deadtime = 10_ns;
  pwmBeginInfo.enable_trig0_interrupt = false;
  pwmBeginInfo.enable_trig1_interrupt = false;
  pwmBeginInfo.trig0 = std::nullopt;
  pwmBeginInfo.trig1 = std::nullopt;
  pwm::begin(pwmBeginInfo);
  pwm::begin();

  while(true) {
  }
}

/* int main() { */
/*   Serial.begin(9600); */
/*  */
/*   xbar::begin(); */
/*  */
/*   PwmBeginInfo pwmBeginInfo; */
/*   pwmBeginInfo.enable_outputs = false; */
/*   pwmBeginInfo.frequency = 20_kHz; */
/*   pwmBeginInfo.deadtime = 10_ns; */
/*   pwmBeginInfo.enable_trig0_interrupt = false; */
/*   pwmBeginInfo.enable_trig1_interrupt = false; */
/*   pwmBeginInfo.trig0 = 0.0f; */
/*   pwmBeginInfo.trig1 = std::nullopt; */
/*   pwm::begin(pwmBeginInfo); */
/*  */
/*   TrigChainInfo chains[2]; */
/*   chains[0].trig_num = TRIG0; */
/*   int chain0_pins[] = {0,1,2}; //TODO use propery pinouts */
/*   chains[0].read_pins = chain0_pins; */
/*   chains[0].chain_priority = 0; */
/*   chains[0].software_trig = false; */
/*   chains[0].trig_sync = true; */
/*   chains[0].intr = DONE0; */
/*  */
/*   chains[1].trig_num = TRIG4; */
/*   int chain4_pins[] = {0,1,2}; */
/*   chains[1].read_pins = chain4_pins; */
/*   chains[1].chain_priority = 0; */
/*   chains[1].software_trig = false; */
/*   chains[1].trig_sync = true; */
/*   chains[1].intr = NONE; */
/*  */
/*   AdcEtcBeginInfo adcBeginInfo; */
/*   adcBeginInfo.adc1_avg = HwAvg::SAMPLE_4; */
/*   adcBeginInfo.adc1_clock_div = AdcClockDivider::NO_DIV; */
/*   adcBeginInfo.adc1_high_speed = true; */
/*   adcBeginInfo.adc1_sample_time = PERIOD_25; */
/*   adcBeginInfo.adc2_avg = adcBeginInfo.adc1_avg; */
/*   adcBeginInfo.adc2_clock_div = adcBeginInfo.adc1_clock_div; */
/*   adcBeginInfo.adc2_high_speed = adcBeginInfo.adc2_high_speed; */
/*   adcBeginInfo.adc2_sample_time = adcBeginInfo.adc2_sample_time; */
/*   adcBeginInfo.num_chains = sizeof(chains) / sizeof(TrigChainInfo); */
/*   adcBeginInfo.chains = chains; */
/*   AdcEtc::begin(adcBeginInfo); */
/*  */
/*   xbar::connect(pwm::TRIG0_SIGNAL_SOURCE, AdcEtc::TRIG0_SIGNAL_SINK); */
/*   xbar::connect(pwm::TRIG1_SIGNAL_SOURCE, AdcEtc::TRIG1_SIGNAL_SINK); */
/*  */
/*   while (true) { */
/*  */
/*     delay(50); */
/*   } */
/* } */
