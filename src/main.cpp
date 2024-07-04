#include "core_pins.h"
#include "firmware/adc_etc.hpp"
#include "firmware/control_transition.hpp"
#include "firmware/pinout.hpp"
#include "firmware/pwm.hpp"
#include "firmware/xbar.hpp"
#include "imxrt.h"
#include "pins_arduino.h"
#include "util/boxcar.h"
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

bool error_flag = false;
int main_counter = 0;


static BoxcarFilter<float, 1000> lim_l(0);
static BoxcarFilter<float, 1000> lim_r(0);
static BoxcarFilter<float, 1000> mag_l(0);
static BoxcarFilter<float, 1000> mag_r(0);

// VDC measurement
float v_dc_raw_meas;
float v_dc;

// displacement parameters
float offset;
float target_offset;
float disp_meas_MAG_L, disp_meas_MAG_R, disp_meas_LIM_L, disp_meas_LIM_R;
float error_disp, error_disp_d;
float integrator_disp = 0;
float derivative_disp;
float i_target; // PWM output (target current)
float i_target_L, i_target_R;

// current parameters
float i_meas_L, i_meas_R;
float error_current;
float error_current_L;
float error_current_R;
float integrator_current_L = 0;
float integrator_current_R = 0;
float v_target_L; // PWM output (target voltage)
float v_target_R; // PWM output (target voltage)

float control_L, control_R;

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

  // get raw values for current and displacement measurements

  i_meas_L = res.trig_res<0, 0>(); // I_MAG_L
  i_meas_R = res.trig_res<0, 1>(); // I_MAG_R
  v_dc_raw_meas = res.trig_res<0, 2>(); // VDC


  disp_meas_MAG_L = res.trig_res<4, 0>(); // DISP_SENS_MAG_L
  disp_meas_MAG_R = res.trig_res<4, 1>(); // DISP_SENS_MAG_R
  disp_meas_LIM_L = res.trig_res<4, 2>(); // DISP_SENS_MAG_L
  disp_meas_LIM_R = res.trig_res<4, 3>(); // DISP_SENS_MAG_R

  // convert to actual values

  // current measurement for LEFT and RIGHT magnets
  i_meas_L = 3.3 * i_meas_L / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_L = -(i_meas_L - 2.5) / GuidanceBoardCurrentGains::GAIN_I_LEFT / 0.001; // current in amps

  i_meas_R = 3.3 * i_meas_R / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_R = -(i_meas_R - 2.5) / GuidanceBoardCurrentGains::GAIN_I_RIGHT / 0.001; // current in amps

  // displacement measurement for LEFT and RIGHT magnets
  disp_meas_MAG_L = 3.3 * disp_meas_MAG_L / (4096.0 * 120); // sense current
  disp_meas_MAG_L = (disp_meas_MAG_L - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm

  disp_meas_MAG_R = 3.3 * disp_meas_MAG_R / 4096.0 / 120.0; // sense current
  disp_meas_MAG_R = (disp_meas_MAG_R - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm

  // displacement measurement for LIM rotor LEFT and RIGHT side
  disp_meas_LIM_L = 3.3 * disp_meas_LIM_L / 4096.0 / 120.0; // sense current
  disp_meas_LIM_L = (disp_meas_LIM_L - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm
  // disp_meas_LIM_L = -disp_meas_LIM_L;

  disp_meas_LIM_R = 3.3 * disp_meas_LIM_R / 4096.0 / 120.0; // sense current
  disp_meas_LIM_R = (disp_meas_LIM_R - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm
  // disp_meas_LIM_R = -disp_meas_LIM_R;

  lim_l.push(disp_meas_LIM_L);
  lim_r.push(disp_meas_LIM_R);
  mag_l.push(disp_meas_MAG_L);
  mag_r.push(disp_meas_MAG_R);

  // VDC measurement (GAIN: theoretically with 1.5, but 1.515 is more accurate)
  v_dc_raw_meas = 3.3 * v_dc_raw_meas / 4096.0;
  // EMA filtering for VDC
  v_dc = 0.9 * v_dc + 0.1 * v_dc_raw_meas * 52500 / (1500 * 1.49 * 0.4); // voltage divider between 1k5 and 51k

  // calculate offset from center of track based on airgaps
  // method: subtract both displacement measurements and divide by two
  // positive offset -> pod shifts to the left -> right magnet needs current
  // negative offset -> pod shifts to the right -> left magnet needs current
  // goal: offset = 0 -> pod stays in the middle
  //
  // LIM airgaps:
  // min:       3 mm
  // nominal:   7 mm
  // max:       11 mm


  // error detection
  /*
  if ((disp_meas_LIM_R < 33 || disp_meas_LIM_R > 39) && (disp_meas_LIM_L < 33 || disp_meas_LIM_L > 39)) {
    // too close -> error 
    digitalWrite(GuidanceBoardPin::SDC_TRIG, LOW); 
    pwm::disable_output(); 
    pwm::trig0(std::nullopt); 
    Serial.printf("ERROR\n"); 
    error_flag = true; 
  } 
  */

  /*
  // error detection for current
  if (i_meas_L < -20 || i_meas_L > 20 || i_meas_R < -20 || i_meas_R > 20) { 
    digitalWrite(GuidanceBoardPin::SDC_TRIG, LOW);
    pwm::disable_output();
    pwm::trig0(std::nullopt);
    Serial.printf("ERROR\n");
    error_flag = true;
  }
  */

  // min MAG displacement : 32.0
  // max MAG displacement: 40.0

  // calibration of the offsets of the sensors
  disp_meas_LIM_R -= 33.02;
  disp_meas_LIM_L -= 32.10;

  disp_meas_MAG_R -= 26.45;
  disp_meas_MAG_L -= 27.6;

  /*
  // EMA filtering (alpha = 0.9) for displacement sensors for magnets
  float disp_meas_MAG_R_delayed = disp_meas_MAG_R;
  float disp_meas_MAG_L_delayed = disp_meas_MAG_L;

  disp_meas_MAG_R = 0.95 * disp_meas_MAG_R + 0.05 * disp_meas_MAG_R_delayed;
  disp_meas_MAG_L = 0.95 * disp_meas_MAG_L + 0.05 * disp_meas_MAG_L_delayed;
  */
  offset = (disp_meas_LIM_L - disp_meas_LIM_R) / 2;
  target_offset = 0;

  //////////////////////////////// distance control ///////////////////////////////////

  if (!error_flag) {

    float P = 15; // P = 1.3 for pod
    float ITs = 0.001; // I = 0.02 for pod
    float D = 0.6; // D = 0.9 for pod 

    error_disp_d = error_disp;
    error_disp = 0.95 * error_disp - 0.05 * (target_offset - offset); // displacement EMA with alpha 0.95
    integrator_disp += error_disp * ITs;
    derivative_disp = D * (error_disp - error_disp_d) * 20000; // 20kHz PWM frequency

    // clamp integrator (min = -30, max = 30)
    if (integrator_disp > 90) {
      integrator_disp = 90;
    } else if (integrator_disp < -90) {
      integrator_disp = -90;
    }

    // PID to calculate target current
    i_target = P * error_disp + integrator_disp + derivative_disp;

    if(i_target >= 0) {
      i_target = std::sqrt(i_target);
    } else {
      i_target = -std::sqrt(-i_target);
    }

    // clamp i_target (min = -40, max = 40)
    if (i_target > 40) {
      i_target = 40;
    } else if (i_target < -40) {
      i_target = -40;
    }

    //////////////////////////////// current control ///////////////////////////////////
    P = 1.3;
    ITs = 0.75;

    // calculate individual target currents
    // i_target_L = 1;
    // i_target_R = 1;

    // when distance at L is bigger, the input to the distance PID is positive
    // hence the right magnet should be (more) active 
    // when i_target (the controller output) is positive
    // some current is always running through the magnets as a baseline (0 amp in this case)

    if(i_target > 0) {
      i_target_R = 0 + i_target;
      i_target_L = 0;
    } else {
      i_target_R = 0;
      i_target_L = 0 - i_target;
    }


    // linearise target currents with magnet distance (seems to make it worse)
    // i_target_L = i_target_L * disp_meas_MAG_L / 5.721381;
    // i_target_R = i_target_R * disp_meas_MAG_R / 6.997266;

    // calculate (filtered) errors
    error_current_L = 0.5 * error_current_L + 0.5 * (i_target_L - i_meas_L); // curent EMA with alpha 0.5
    error_current_R = 0.5 * error_current_R + 0.5 * (i_target_R - i_meas_R); // current EMA with alpha 0.5
    
    // integrator
    integrator_current_L += error_current_L * ITs;
    integrator_current_R += error_current_R * ITs;

    // clamp integrator (min = -10, max = 10)
    if (integrator_current_L > 20) {
      integrator_current_L = 20;
    } else if (integrator_current_L < -20) {
      integrator_current_L = -20;
    }
    if (integrator_current_R > 20) {
      integrator_current_R = 20;
    } else if (integrator_current_R < -20) {
      integrator_current_R = -20;
    }

    // PI control
    v_target_L = P * error_current_L + integrator_current_L;
    v_target_R = P * error_current_R + integrator_current_R;

    // clamp target voltage (min = 0, max = 40)
    if (v_target_L > 40) {
      v_target_L = 40;
    } else if (v_target_L < -20) {
      v_target_L = -20;
    }
    if (v_target_R > 40) {
      v_target_R = 40;
    } else if (v_target_R < -20) {
      v_target_R = -20;
    }

    // set output of target voltage to PWM
    control_L = v_target_L / v_dc;
    control_R = v_target_R / v_dc;

    if (i_meas_L < 1 && i_target_L <= 0) {
      control_L = 0;
    }
    
    if (i_meas_R < 1 && i_target_R <= 0) {
      control_R = 0;
    }

    // limit the duty cycle  
    if (control_L > 0.9) { 
      control_L = 0.9;
    }
    if (control_L < -0.9) { 
      control_L = -0.9;
    }
    
    if (control_R > 0.9) { 
      control_R = 0.9;
    }
    if (control_R < -0.9) { 
      control_R = -0.9;
    }
    
    // initial test
    // control_R = 0.0;
    // control_L = 0.0;

    /*
    0.1 Duty 2.65A
    0.125 Duty 3.6A
    0.13 Duty 3.72A
    0.14 Duty 4.12A
    0.15 Duty 4.51A
    0.16 Duty 4.91A
    0.2 Duty 6.45A
    */

    // write outputs
    PwmControl isr_control;
    isr_control.duty13 = 0.5 + control_R / 2; // RIGHT_R
    isr_control.duty31 = 0.5 - control_R / 2; // RIGHT_L

    isr_control.duty42 = 0.5 + control_L / 2; // LEFT_R
    isr_control.duty22 = 0.5 - control_L / 2; // LEFT_L

    pwm::control(isr_control);
  }
}

void adc_etc_done1_isr(AdcTrigRes res) {
  // g_done1_timing.tick();
}

int main() {
  Serial.begin(9600);

  error_flag = false;

  pinMode(GuidanceBoardPin::SDC_TRIG, OUTPUT);
  pinMode(GuidanceBoardPin::PRECHARGE_DONE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.printf("Guidance Test Start\n");

  PwmBeginInfo pwmBeginInfo;
  pwmBeginInfo.enable_outputs = true;
  pwmBeginInfo.frequency = 20_kHz;
  pwmBeginInfo.deadtime = 1000_ns;
  pwmBeginInfo.enable_trig0_interrupt = false;
  pwmBeginInfo.enable_trig1_interrupt = false;
  pwmBeginInfo.trig0 = 0.0f;
  pwmBeginInfo.trig1 = 0.0f;
  pwm::begin(pwmBeginInfo);

  TrigChainInfo chains[2];
  chains[0].trig_num = TRIG0;
  int chain0_pins[] = {GuidanceBoardPin::I_MAG_L,
                       GuidanceBoardPin::I_MAG_R,
                       GuidanceBoardPin::VDC_MEAS};
  chains[0].read_pins = chain0_pins;
  chains[0].chain_length = sizeof(chain0_pins) / sizeof(int);
  chains[0].chain_priority = 0;
  chains[0].software_trig = false;
  chains[0].trig_sync = true;
  chains[0].intr = NONE;

  chains[1].trig_num = TRIG4;
  int chain4_pins[] = {GuidanceBoardPin::DISP_SENS_MAG_L,
                       GuidanceBoardPin::DISP_SENS_MAG_R,
                       GuidanceBoardPin::DISP_SENS_LIM_L,
                       GuidanceBoardPin::DISP_SENS_LIM_R};
  chains[1].read_pins = chain4_pins;
  chains[1].chain_length = sizeof(chain4_pins) / sizeof(int);
  chains[1].chain_priority = 0;
  chains[1].software_trig = false;
  chains[1].trig_sync = false;
  chains[1].intr = DONE0;


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
  /* xbar::connect(pwm::TRIG1_SIGNAL_SOURCE, AdcEtc::TRIG1_SIGNAL_SINK); */

  delay(5000);
  Serial.println("Enable Not-Aus");
  delay(5000);
  Serial.println("Enable outputs");
  if (!error_flag) {
    pwm::enable_output();
  }

  // precharge
  delay(1000);
  digitalWrite(GuidanceBoardPin::SDC_TRIG, HIGH);
  delay(1000);
  digitalWrite(GuidanceBoardPin::PRECHARGE_START, HIGH);
  delay(2000);
  digitalWrite(GuidanceBoardPin::PRECHARGE_DONE, HIGH);
  digitalWrite(GuidanceBoardPin::PRECHARGE_START, LOW);
  delay(1000);

  while (true) {
    if (error_flag) {
      digitalWrite(LED_BUILTIN, HIGH);
      pwm::disable_output();
    }
    // Serial.printf("I_left: %f  -  I_right: %f  -  DISP_LIM_left: %f  -
    // DISP_LIM_right: %f \n",
    //               i_meas_L, i_meas_R, disp_meas_LIM_L, disp_meas_LIM_R);
    // Serial.printf("I_right: %f  -  Error: %f  -  I_Target: %f  -  DISP_right:
    // %f \n",
    //                 i_meas_R, in_d, i_target, disp_meas_MAG_R);

    // Serial.printf("Disp_Targ: %f - I_Targ: %f - Integr Disp: %f - DISP_R: %f
    // - Out: %f \n",
    //                 disp_target, i_target, integrator_disp, disp_meas_MAG_R,
    //                 out);
    // disp_target -= 0.2 / 20;
    // if(disp_target < 6) {
    //   disp_target = 6;
    // }
    // digitalWrite(LED_BUILTIN, LOW);


    /*
    Serial.printf("error flag %u\n", error_flag);
  
    Serial.printf("airgap_left %f \nairgap_right %f\n", lim_l.get(), lim_r.get());

    Serial.printf(
        "Measured Offset: %f - Target Current: %f - Target Voltage: %f \n ",
        offset, i_target, v_target);
    */

    Serial.printf("Control_L: %f - Control_R: %f \n",
                    control_L, control_R);

    Serial.printf("Current left: %f - Current right: %f  - Target current: %f \n",
                    i_meas_L, i_meas_R, i_target);

    Serial.printf("Disp_LIM_L: %f  -  Disp_LIM_R: %f  -  Disp_MAG_L: %f  -  Disp_MAG_R: %f \n\n",
                  disp_meas_LIM_L, disp_meas_LIM_R, disp_meas_MAG_L, disp_meas_MAG_R);

    delay(200);
  }
}
