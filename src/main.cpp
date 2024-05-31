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

// define clamp function (use to refactor code, once clamp in code works, e.g.
// clamp integrator_disp, clamp i_target, etc.)
float clamp(float in, float min, float max) {
  if (in > max) {
    return max;
  }
  if (in < min) {
    return min;
  }
  return in;
}

// displacement parameters
float offset;
float target_offset;
float disp_meas_MAG_L, disp_meas_MAG_R, disp_meas_LIM_L, disp_meas_LIM_R;
float error_disp, error_disp_d;
float integrator_disp;
float derivative_disp;
float i_target; // PWM output (target current)

// current parameters
float i_meas_L, i_meas_R;
float error_current;
float error_current_L;
float error_current_R;
float integrator_current;
float v_target; // PWM output (target voltage)

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

  // configuration 1

  i_meas_L = res.trig_res<0, 0>(); // I_MAG_L
  i_meas_R = res.trig_res<0, 1>(); // I_MAG_R

  disp_meas_MAG_L = res.trig_res<4, 0>(); // DISP_SENS_MAG_L
  disp_meas_MAG_R = res.trig_res<4, 1>(); // DISP_SENS_MAG_R
  disp_meas_LIM_L = res.trig_res<4, 2>(); // DISP_SENS_MAG_L
  disp_meas_LIM_R = res.trig_res<4, 3>(); // DISP_SENS_MAG_R

  /*

  configuration 2

  i_meas_L = res.trig_res<0, 0>(); // I_MAG_L
  i_meas_R = res.trig_res<4, 0>(); // I_MAG_R

  disp_meas_MAG_L = res.trig_res<0, 1>(); // DISP_SENS_MAG_L
  disp_meas_MAG_R = res.trig_res<4, 1>(); // DISP_SENS_MAG_R

  disp_meas_LIM_L = res.trig_res<0, 2>(); // DISP_SENS_LIM_L
  disp_meas_LIM_R = res.trig_res<4, 2>(); // DISP_SENS_LIM_R


  */

  // convert to actual values

  // current measurement for LEFT and RIGHT magnets
  i_meas_L = 3.3 * i_meas_L / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_L = (i_meas_L - 2.5) / GuidanceBoardCurrentGains::GAIN_I_LEFT / 0.001; // current in amps

  i_meas_R = 3.3 * i_meas_R / 4096.0 / 1.5 / 0.4; // voltage at input of isolation amplifier
  i_meas_R = -(i_meas_R - 2.5) / GuidanceBoardCurrentGains::GAIN_I_RIGHT / 0.001; // current in amps

  // displacement measurement for LEFT and RIGHT magnets
  disp_meas_MAG_L = 3.3 * disp_meas_MAG_L / (4096.0 * 120); // sense current
  disp_meas_MAG_L = (disp_meas_MAG_L - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm

  disp_meas_MAG_R = 3.3 * disp_meas_MAG_R / 4096.0 / 120.0; // sense current
  disp_meas_MAG_R = (disp_meas_MAG_R - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm

  // displacement measurement for LIM rotor LEFT and RIGHT side
  disp_meas_LIM_L = 3.3 * disp_meas_LIM_L / 4096.0 / 120; // sense current
  disp_meas_LIM_L = (disp_meas_LIM_L - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm

  disp_meas_LIM_R = 3.3 * disp_meas_LIM_R / 4096.0 / 120.0; // sense current
  disp_meas_LIM_R = (disp_meas_LIM_R - 0.004) * (50.0 - 20.0) / 0.016 + 20.0; // map 4...20mA to 20...50mm

  lim_l.push(disp_meas_LIM_L);
  lim_r.push(disp_meas_LIM_R);

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
  /* if ((disp_meas_LIM_R < 33 || disp_meas_LIM_R > 39) && */
  /*     (disp_meas_LIM_L < 33 || disp_meas_LIM_L > 39)) { */
  /*   // too close -> error */
  /*   digitalWrite(GuidanceBoardPin::SDC_TRIG, LOW); */
  /*   pwm::disable_output(); */
  /*   pwm::trig0(std::nullopt); */
  /*   Serial.printf("ERROR\n"); */
  /*   error_flag = true; */
  /* } */

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

  disp_meas_LIM_R -= 33.02;
  disp_meas_LIM_L -= 32.10;

  offset = (disp_meas_LIM_L - disp_meas_LIM_R) / 2;
  target_offset = 0;

  //////////////////////////////// distance control ///////////////////////////////////

  if (!error_flag) {

    float P = 1.2; // P = 1.3 for pod
    float ITs = 0.01; // I = 0.02 for pod
    float D = 0.33; // D = 0.9 for pod 

    error_disp_d = error_disp;
    error_disp = 0.9 * error_disp - 0.1 * (target_offset - offset); // displacement EMA with alpha 0.9
    integrator_disp += error_disp * ITs;
    derivative_disp = D * (error_disp - error_disp_d) * 20000; // 20kHz PWM frequency

    // clamp integrator (min = -30, max = 30)
    if (integrator_disp > 30) {
      integrator_disp = 30;
    } else if (integrator_disp < -30) {
      integrator_disp = -30;
    }

    // PID to calculate target current
    i_target = P * error_disp + integrator_disp + derivative_disp;

    // clamp i_target (min = -40, max = 40)
    if (i_target > 40) {
      i_target = 40;
    } else if (i_target < -40) {
      i_target = -40;
    }

    //////////////////////////////// current control ///////////////////////////////////
    P = 1.3;
    ITs = 0.75;

    // is this part correct? (-i_target)
    error_current_L = 0.5 * error_current_L + 0.5 * (-i_target - i_meas_L); // curent EMA with alpha 0.5 (invert i_target)
    error_current_R = 0.5 * error_current_R + 0.5 * (i_target - i_meas_R); // current EMA with alpha 0.5

    // Switch case to select error current based on polarity of output PID
    // (i_target)
    if (i_target > 0) {
      error_current = error_current_R;

    } else if (i_target < 0) {
      error_current = error_current_L;

    } else {
      error_current = 0;
    }

    integrator_current += error_current * ITs;

    // clamp integrator (min = -10, max = 10)
    if (integrator_current > 10) {
      integrator_current = 10;
    } else if (integrator_current < -10) {
      integrator_current = -10;
    }

    // PID to calculate target voltage
    v_target = P * error_current + integrator_current;

    // clamp target voltage (min = 0, max = 40)
    if (v_target > 40) {
      v_target = 40;
    } else if (v_target < 0) {
      v_target = 0;
    }

    i_target = 0;
    v_target = 0;

    // set output of target voltage to PWM (is this part correct?)
    if (i_target > 0) {
      control_R = v_target / 45;
      control_L = 0;

    } else if (i_target < 0) {
      control_L = v_target / 45;
      control_R = 0;

    } else {
      control_L = 0;
      control_R = 0;
    }

    // limit the duty cycle  
    if (control_L > 0.9) { 
      control_L = 0.9;
    }
    if (control_R > 0.9) { 
      control_R = 0.9;
    }
    

    // initial test
    control_R = 0.0;
    control_L = 0.0;

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

  Serial.println("Enable Not-Aus");

  /*

  configuration 1

  TrigChainInfo chains[2];
  chains[0].trig_num = TRIG0;
  int chain0_pins[] = {GuidanceBoardPin::I_MAG_L,
                       GuidanceBoardPin::DISP_SENS_MAG_L,
                       GuidanceBoardPin::DISP_SENS_LIM_L};
  chains[0].read_pins = chain0_pins;
  chains[0].chain_length = sizeof(chain0_pins) / sizeof(int);
  chains[0].chain_priority = 0;
  chains[0].software_trig = false;
  chains[0].trig_sync = true;
  chains[0].intr = DONE0;

  chains[1].trig_num = TRIG4;
  int chain4_pins[] = {GuidanceBoardPin::I_MAG_R,
                       GuidanceBoardPin::DISP_SENS_MAG_R,
                       GuidanceBoardPin::DISP_SENS_LIM_R};
  chains[1].read_pins = chain4_pins;
  chains[1].chain_length = sizeof(chain4_pins) / sizeof(int);
  chains[1].chain_priority = 0;
  chains[1].software_trig = false;
  chains[1].trig_sync = false;
  chains[1].intr = NONE;

  */

  // configuration 2

  TrigChainInfo chains[2];
  chains[0].trig_num = TRIG0;
  int chain0_pins[] = {GuidanceBoardPin::I_MAG_L,
                       GuidanceBoardPin::I_MAG_R};
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
  // "precharge"
  delay(1000);
  digitalWrite(GuidanceBoardPin::SDC_TRIG, HIGH);
  delay(1000);
  digitalWrite(GuidanceBoardPin::PRECHARGE_DONE, HIGH);
  delay(1000);
  Serial.println("Enable outputs");
  if (!error_flag) {
    pwm::enable_output();
  }


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

    Serial.printf("Control_L: %f - Control_R: %f \n ",
                    control_L, control_R);

    Serial.printf("Current left: %f - Current right: %f  - Target current: %f \n \n",
                    i_meas_L, i_meas_R, i_target);

    delay(50);
  }
}
