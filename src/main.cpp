

#include "core_pins.h"
#include "firmware/adc_etc.hpp"
#include "firmware/pwm.hpp"
#include "firmware/xbar.hpp"
#include "firmware/pinout.hpp"
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


float i_meas_L, i_meas_R, disp_meas_MAG_L, disp_meas_MAG_R;

float out, out_d, out_dd, in, in_d, in_dd; // delayed outputs


void pwm_trig0_isr() {
  // g_adc_timing.start();
  // g_trig_timing.start();
  // Not used right now
}


void pwm_trig1_isr() {
  // g_trig_time_filter.push(static_cast<Time>(g_trig_timing.time()));
  // Not used right now
}


void adc_etc_done0_isr(AdcTrigRes res) {
  // g_adc_time_filter.push(static_cast<Time>(g_adc_timing.time()));
  // g_done0_timing.tick();

  // get raw values
  i_meas_L = res.trig_res<0,0>(); // I_MAG_L
  i_meas_R = res.trig_res<0,1>(); // I_MAG_R
  
  disp_meas_MAG_L = res.trig_res<4,0>(); // DISP_SENS_MAG_L
  disp_meas_MAG_R = res.trig_res<4,1>(); // DISP_SENS_MAG_R

  // convert to actual values
  i_meas_L = 3.3 * i_meas_L / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_L = (i_meas_L - 2.5) / GuidanceBoardCurrentGains::GAIN_I_LEFT / 0.001; // current in amps

  i_meas_R = 3.3 * i_meas_R / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_R = -(i_meas_R - 2.5) / GuidanceBoardCurrentGains::GAIN_I_RIGHT / 0.001; // current in amps

  disp_meas_MAG_L = 3.3 * disp_meas_MAG_R / 4096.0 / 120; // sense current
  disp_meas_MAG_L = (disp_meas_MAG_L - 0.004) * (50.0 - 25.0) / 0.016 + 25.0; // map 4...20mA to 25...50mm

  disp_meas_MAG_R = 3.3 * disp_meas_MAG_R / 4096.0 / 120.0; // sense current
  disp_meas_MAG_R = (disp_meas_MAG_R - 0.004) * (50.0 - 25.0) / 0.016 + 25.0; // map 4...20mA to 25...50mm

  // current control
  float P = 0.2;
  float ITs = 0;
  float D = 0;
  float N = 10;
  float i_target = 2;
  float Ts = 1/20000.0;

  float A_coef = N*Ts - 2;
  float B_coef = 1 - N*Ts;
  float C_coef = P + D*N;
  float D_coef = P*(N*Ts - 2) - 2*D*N;
  float E_coef = P*(1-N*Ts) + ITs*N*Ts + D*N;

  float error = i_target - i_meas_R;

  in_dd = in_d;
  in_d = in;
  out_dd = out_d;
  out_d = out;

  in = error;
  // out = C_coef*in + D_coef*in_d + E_coef*in_dd - A_coef*out_d - B_coef*out_dd;

  out = error*4;
  if(out > 22.5) {
    out = 22.5;
  } else if(out < 0) {
    out = 0;
  }

  float control = out/22.5;


  // distance control


  // write outputs
  PwmControl isr_control;
  isr_control.duty13 = 0.5 + control/2;
  isr_control.duty31 = 0.5 - control/2;
  pwm::control(isr_control);

}


void adc_etc_done1_isr(AdcTrigRes res) {
  // g_done1_timing.tick();
}




int main() {

  Serial.begin(9600);

  pinMode(GuidanceBoardPin::SDC_TRIG, OUTPUT);
  pinMode(GuidanceBoardPin::PRECHARGE_DONE, OUTPUT);

  /* delay(2000); */
  Serial.printf("Hello, World\n");

  PwmBeginInfo pwmBeginInfo;
  pwmBeginInfo.enable_outputs = false;
  pwmBeginInfo.frequency = 20_kHz;
  pwmBeginInfo.deadtime = 1000_ns;
  pwmBeginInfo.enable_trig0_interrupt = true;
  pwmBeginInfo.enable_trig1_interrupt = false;
  pwmBeginInfo.trig0 = 0.0f;
  pwmBeginInfo.trig1 = 0.25f;
  pwm::begin(pwmBeginInfo);

  TrigChainInfo chains[3];
  chains[0].trig_num = TRIG0;
  int chain0_pins[] = {GuidanceBoardPin::I_MAG_L,
                        GuidanceBoardPin::I_MAG_R};
  chains[0].read_pins = chain0_pins;
  chains[0].chain_length = 2;
  chains[0].chain_priority = 0;
  chains[0].software_trig = false;
  chains[0].trig_sync = true;
  chains[0].intr = DONE0;

  chains[1].trig_num = TRIG4;
  int chain4_pins[] = {GuidanceBoardPin::DISP_SENS_MAG_L,
                        GuidanceBoardPin::DISP_SENS_MAG_R};
  chains[1].read_pins = chain4_pins;
  chains[1].chain_length = 2;
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
  adcBeginInfo.adc1_sample_time = PERIOD_17;
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


  // "precharge"
  delay(1000);
  digitalWrite(GuidanceBoardPin::SDC_TRIG, HIGH);
  delay(1000);
  digitalWrite(GuidanceBoardPin::PRECHARGE_DONE, HIGH);
  delay(1000);
  pwm::enable_output();

  while (true) {
    // Serial.printf("DONE0 Frequency = %fHz, DONE1 Frequency = %fHz\n", static_cast<float>(g_done0_timing.frequency()),
    //     static_cast<float>(g_done1_timing.frequency()));
    // Serial.printf("ADC Convertion time = %fus\n", static_cast<float>(g_adc_time_filter.get()) * 1e6);
    // Serial.printf("Time between triggers = %fus\n", static_cast<float>(g_trig_time_filter.get()) * 1e6);
    
    Serial.printf("I_left: %f  -  I_right: %f  -  DISP_left: %f  -  DISP_right: %f \n",
                    i_meas_L, i_meas_R, disp_meas_MAG_L, disp_meas_MAG_R);

    delay(50);
  }
}

