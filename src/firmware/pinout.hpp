#pragma once

#include <inttypes.h>

enum GuidanceBoardPin : uint8_t {
  
  // Precharge 
  PRECHARGE_START = 32,
  PRECHARGE_DONE = 31,

  // SDC trigger pin
  SDC_TRIG = 37,
  
  // Displacement sensors (analog)
  DISP_SENS_MAG_L = 19,
  DISP_SENS_LIM_L = 18,
  DISP_SENS_MAG_R = 17,
  DISP_SENS_LIM_R = 16,
  
  // NTCs (analog)
  TEMPSENS_L2 = 21,
  TEMPSENS_L1 = 20,
  TEMPSENS_R2 = 15,
  TEMPSENS_R1 = 14,
  
  // V_DC measurement (analog)
  VDC_MEAS = 40,

  // Current measurement (analog)
  I_MAG_L = 24,
  I_MAG_R = 25,
  I_MAG_TOTAL = 26,

  // LEFT Magnet
  LEFT_HIN_L = 2, // HIGH
  LEFT_LIN_L = 3,  // LOW

  LEFT_HIN_R = 6, // HIGH
  LEFT_LIN_R = 9, // LOW

  // RIGHT Magnet
  RIGHT_HIN_L = 8, // HIGH
  RIGHT_LIN_L = 7,  // LOW

  RIGHT_HIN_R = 29, // HIGH
  RIGHT_LIN_R = 28, // LOW
};


enum GuidanceBoardCurrentGains : uint8_t {
  GAIN_I_LEFT = 20,
  GAIN_I_RIGHT = 40,

};

//////////////////////////// PWM Pins ////////////////////////////////

///// LEFT MAGNET //////

// FlexPWM4 Module2
// LEFT_HIN_L: Pin2 HIGH
// LEFT_LIN_L: Pin3 LOW

// FlexPWM2 Module2
// LEFT_HIN_R: Pin6 HIGH
// LEFT_LIN_R: Pin9 LOW

///// RIGHT MAGNET //////

// FlexPWM1 Module3
// RIGHT_HIN_L: Pin8 HIGH
// RIGHT_LIN_L: Pin7 LOW

// FlexPWM1 Module3
// RIGHT_HIN_R: Pin29 HIGH
// RIGHT_LIN_R: Pin28 LOW

