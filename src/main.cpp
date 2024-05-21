

#include "core_pins.h"
#include "firmware/adc_etc.hpp"
#include "firmware/pwm.hpp"
#include "firmware/xbar.hpp"
#include "firmware/pinout.hpp"
#include "imxrt.h"
#include "util/ema.hpp"
#include "util/timestamp.hpp"
#include "util/timimg.hpp"
#include "firmware/control_transition.hpp"
#include <Arduino.h>

#define MONITOR_BUFFER_LENGTH 10000

volatile IntervalTiming g_done0_timing;
volatile IntervalTiming g_done1_timing;

volatile Timing g_adc_timing;
ExponentialMovingAverage<Time> g_trig_time_filter(0.1, 1_s);
volatile Timing g_trig_timing;
ExponentialMovingAverage<Time> g_adc_time_filter(0.1, 1_s);

volatile bool error_flag = false;
volatile bool monitor_flag = false;
volatile bool monitor_done_flag = false;

volatile float monitor_buffer[MONITOR_BUFFER_LENGTH];
volatile int monitor_counter = 0;
volatile int monitor_subcounter = 0;
int error_counter = 0;

int main_counter = 0;


float i_meas_L, i_meas_R, disp_meas_MAG_L, disp_meas_MAG_R;
float disp_filtered_MAG_R;

float out, out_d, out_dd, in, in_d, in_dd, error; // delayed in/outputs current
float error_disp, error_disp_d;
float integrator, integrator_disp;
float state_deriv;
float i_target;
float disp_target;


float clamp(float in, float min, float max) {
  if(in > max) {
    return max;
  }
  if(in < min) {
    return min;
  }
  return in;
}


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
  // i_meas_L = res.trig_res<0,0>(); // I_MAG_L
  i_meas_R = res.trig_res<0,0>(); // I_MAG_R
  
  // disp_meas_MAG_L = res.trig_res<4,0>(); // DISP_SENS_MAG_L
  disp_meas_MAG_R = res.trig_res<4,0>(); // DISP_SENS_MAG_R

  // convert to actual values
  i_meas_L = 3.3 * i_meas_L / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_L = (i_meas_L - 2.5) / GuidanceBoardCurrentGains::GAIN_I_LEFT / 0.001; // current in amps

  i_meas_R = 3.3 * i_meas_R / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_R = -(i_meas_R - 2.5) / GuidanceBoardCurrentGains::GAIN_I_LEFT / 0.001; // current in amps

  disp_meas_MAG_L = 3.3 * disp_meas_MAG_R / 4096.0 / 120; // sense current
  disp_meas_MAG_L = (disp_meas_MAG_L - 0.004) * (50.0 - 25.0) / 0.016 + 25.0; // map 4...20mA to 25...50mm

  disp_meas_MAG_R = 3.3 * disp_meas_MAG_R / 4096.0 / 120.0; // sense current
  disp_meas_MAG_R = (disp_meas_MAG_R - 0.004) * (50.0 - 25.0) / 0.016 + 25.0; // map 4...20mA to 25...50mm
  disp_meas_MAG_R = disp_meas_MAG_R - 29;

  disp_filtered_MAG_R = 0.9 * disp_filtered_MAG_R + 0.1 * disp_meas_MAG_R;

  // error detection
  if(disp_filtered_MAG_R < 3 && disp_filtered_MAG_R > 0) {
    // too close -> error
    error_counter++;
    if(error_counter > 100) {
      digitalWrite(GuidanceBoardPin::SDC_TRIG, LOW);
      pwm::disable_output();

      // reset control to default
      PwmControl error_control;
      pwm::control(error_control);
      error_flag = true;
    }
  } else {
    if(error_counter > 0) {
      error_counter--;
    }
  }

  // transition handling
  disp_target = airgap_transition::step();

  // distance control

  if(!error_flag) {

  float P = 10; // 9
  // float ITs = 0.0005 - (disp_target-7)*0.0001;
  float ITs = 0.004; // 0.002
  float D = 0.045; // 0.02
  float N = 1000;
  // disp_target = 7.4; // temporary
  error_disp_d = error_disp;

  error_disp = 0.9 * error_disp - 0.1 * (disp_target - disp_meas_MAG_R);
  // if(error_disp > 0.9) {
  //   error_disp = 0.9;
  // }
  // if(error_disp < -0.9) {
  //   error_disp = -0.9;
  // }

  integrator_disp += error_disp * ITs;

  // float derivative_disp = D * (error_disp - error_disp_d) * 20000; // standard implementation of derivative
  float derivative_disp = (D * error_disp - state_deriv) * N; // filtered implementation of derivative
  state_deriv += derivative_disp / 20000;

  if(integrator_disp < 0) {
    integrator_disp = 0;
  }else if(integrator_disp > 10) {
    integrator_disp = 10;
  }

  i_target = P*error_disp + integrator_disp + derivative_disp;
  if(i_target > 0) {
    i_target = 2.64575 * sqrt(i_target);
    i_target = i_target * (disp_filtered_MAG_R) / 6;
  }
  


  if(i_target > 50) {
    i_target = 50;
    // digitalWrite(LED_BUILTIN, HIGH);
  }else if(i_target < -4) {
    i_target = -4;
    // digitalWrite(LED_BUILTIN, HIGH);
  }

  // i_target = i_target / 0.75;

  // current control
  P = 8; // 4
  ITs = 0.4; // 0.06
  // float i_target = 5.2 / 0.75; // temporary

  in_d = in;

  error = 0.5 * error + 0.5 * (i_target / 0.75 - i_meas_R); // EMA
  in = error;

  integrator = integrator + clamp(in * ITs, -0.05, 0.05);
  if(integrator > 10) {
    integrator = 10;
  }else if(integrator < -10) {
    integrator = -10;
  }
  
  out = P*in + integrator;
  // out = error*4;
  if(out > 40) {
    out = 40;
  }
  if(i_meas_R > 0.5) {
    if(out < -40) {
      out = -40;
    }
  }else{
    if(out < 0) {
      out = 0;
    }
  }

  float control = out/45;


  // write outputs
  PwmControl isr_control;
  // isr_control.duty13 = 0.5 + control/2;
  // isr_control.duty31 = 0.5 - control/2;
  isr_control.duty42 = 0.5 + control/2;
  isr_control.duty22 = 0.5 - control/2;
  pwm::control(isr_control);
  }

  
  if(monitor_flag) {
    if(monitor_subcounter == 0) {
      monitor_buffer[monitor_counter] = i_target;
      monitor_counter++;
      monitor_buffer[monitor_counter] = i_meas_R;
      monitor_counter++;
      monitor_buffer[monitor_counter] = disp_meas_MAG_R;
      monitor_counter++;
      monitor_buffer[monitor_counter] = error_disp;
      monitor_counter++;
      monitor_buffer[monitor_counter] = out;
      monitor_counter++;

      if(monitor_counter >= MONITOR_BUFFER_LENGTH) {
        monitor_flag = false;
        monitor_done_flag = true;
      }
    }
    monitor_subcounter++;
    if(monitor_subcounter >= 2) {
      monitor_subcounter = 0;
    }
  }

}


void adc_etc_done1_isr(AdcTrigRes res) {
  // g_done1_timing.tick();
}




int main() {
  disp_target = 10;

  Serial.begin(9600);

  pinMode(GuidanceBoardPin::SDC_TRIG, OUTPUT);
  pinMode(GuidanceBoardPin::PRECHARGE_DONE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(5000);
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

  Serial.println("enable not-aus");

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

  // init transition
  airgap_transition::init(10);
  pwm::enable_output();
  
  delay(5000),
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
    
    // Serial.printf("I_left: %f  -  I_right: %f  -  DISP_left: %f  -  DISP_right: %f \n",
    //                 i_meas_L, i_meas_R, disp_meas_MAG_L, disp_meas_MAG_R);
    // Serial.printf("I_right: %f  -  Error: %f  -  I_Target: %f  -  DISP_right: %f \n",
    //                 i_meas_R, in_d, i_target, disp_meas_MAG_R);
    
    // Serial.printf("Disp_Targ: %f - I_Targ: %f - Integr Disp: %f - DISP_R: %f - Out: %f \n",
    //                 disp_target, i_target, integrator_disp, disp_meas_MAG_R, out);
    Serial.printf("Disp_Targ: %f - DISP_R: %f - Out: %f - mon-cnt: %u\n",
                    disp_target, disp_meas_MAG_R, out, monitor_counter);

    // disp_target -= 0.2 / 20;
    // if(disp_target < 6) {
    //   disp_target = 6;
    // }
    // digitalWrite(LED_BUILTIN, LOW);
    if(monitor_done_flag) {
      for(int i=0; i<MONITOR_BUFFER_LENGTH; i++) {
        Serial.printf("%f,", monitor_buffer[i]);
      }
      while(1); //lock
    }

    if(main_counter == 100) {
      // after 5s
      digitalWrite(LED_BUILTIN, LOW);
      airgap_transition::start_transition(6, 8);
    }
    if(main_counter == 300) {
      digitalWrite(LED_BUILTIN, LOW);
      // monitor_flag = true;
      airgap_transition::start_transition(6, 0);
      main_counter = 0;
    }

    main_counter++;
    delay(50);
  }
}

