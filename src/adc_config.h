#pragma once

#include "firmware/adc_etc.h"

static void adc_config() {
  TrigChainInfo chains[2];
  chains[0].trig_num = TRIG0;
  ain_pin chain0_pins[] = {
      ain_pin::i_mag_l_24,          ain_pin::i_mag_l_24,
      ain_pin::i_mag_l_24,          ain_pin::i_mag_l_24,
      ain_pin::i_mag_l_24,          ain_pin::i_mag_l_24,
      ain_pin::disp_sense_lim_l_18, ain_pin::disp_sense_lim_r_16,
  };
  chains[0].read_pins = chain0_pins;
  chains[0].chain_length = sizeof(chain0_pins) / sizeof(ain_pin);
  chains[0].chain_priority = 0;
  chains[0].software_trig = false;
  chains[0].trig_sync = true;
  chains[0].intr = DONE0;

  chains[1].trig_num = TRIG4;
  ain_pin chain4_pins[] = {
      ain_pin::i_mag_r_25,          ain_pin::i_mag_r_25,
      ain_pin::i_mag_r_25,          ain_pin::i_mag_r_25,
      ain_pin::i_mag_r_25,          ain_pin::i_mag_r_25,
      ain_pin::disp_sense_mag_l_19, ain_pin::disp_sense_mag_r_17,
  };
  chains[1].read_pins = chain4_pins;
  chains[1].chain_length = sizeof(chain4_pins) / sizeof(ain_pin);
  chains[1].chain_priority = 0;
  chains[1].software_trig = false;
  chains[1].trig_sync = false;
  chains[1].intr = NONE;

  AdcEtcBeginInfo adcBeginInfo;
  adcBeginInfo.adc1_avg = HwAvg::SAMPLE_8;
  adcBeginInfo.adc1_clock_div = AdcClockDivider::NO_DIV;
  adcBeginInfo.adc1_high_speed = true;
  adcBeginInfo.adc1_sample_time = PERIOD_25;
  adcBeginInfo.adc2_avg = adcBeginInfo.adc1_avg;
  adcBeginInfo.adc2_clock_div = adcBeginInfo.adc1_clock_div;
  adcBeginInfo.adc2_high_speed = adcBeginInfo.adc2_high_speed;
  adcBeginInfo.adc2_sample_time = adcBeginInfo.adc2_sample_time;
  adcBeginInfo.chains = chains;
  adcBeginInfo.num_chains = sizeof(chains) / sizeof(TrigChainInfo);

  adc_etc::begin(adcBeginInfo);
}
