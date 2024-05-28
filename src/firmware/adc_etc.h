#pragma once
#include "firmware/pinout.h"
#include "util/metrics.h"
#include <cinttypes>

enum DoneInterrupt:uint8_t {
  NONE = 0b00,
  DONE0 = 0b01,
  DONE1 = 0b10,
  DONE2 = 0b11
};

enum HwAvg:uint8_t {
  SAMPLE_4 = 0b00,
  SAMPLE_8 = 0b01,
  SAMPLE_16 = 0b10,
  SAMPLE_32 = 0b11,
};

enum SampleTime:uint8_t {
  PERIOD_3 = 0b000,
  PERIOD_5 = 0b001,
  PERIOD_7 = 0b010,
  PERIOD_9 = 0b011,
  PERIOD_13 = 0b100,
  PERIOD_17 = 0b101,
  PERIOD_21 = 0b110,
  PERIOD_25 = 0b111,
};

enum AdcResolution:uint8_t {
  BIT_8 = 0b00,
  BIT_10 = 0b01,
  BIT_12 = 0b10,
};

enum AdcClockDivider:uint8_t {
  NO_DIV = 0b00,
  DIV_2 = 0b01,
  DIV_4 = 0b10,
  DIV_8 = 0b11
};

// Trigger 0-3 use adc1, tigger 4-7 use adc2
// Trigger 3 and 7 are reserved for software manual analogRead-style reads
enum AdcEtcTrigger:uint8_t {
  TRIG0 = 0,
  TRIG1 = 1,
  TRIG2 = 2,
  TRIG4 = 4,
  TRIG5 = 5,
  TRIG6 = 6,
};


struct TrigChainInfo {
  AdcEtcTrigger trig_num = TRIG0;
  int chain_length = 1;
  ain_pin* read_pins = nullptr;
  bool trig_sync = false;
  int chain_priority = 0;
  bool software_trig = false;
  DoneInterrupt intr = NONE;
};


struct AdcEtcBeginInfo {
  int num_chains = 0;
  TrigChainInfo* chains = nullptr;
  HwAvg adc1_avg = SAMPLE_4;
  HwAvg adc2_avg = SAMPLE_4;
  bool adc1_high_speed = false;
  bool adc2_high_speed = false;
  SampleTime adc1_sample_time = PERIOD_7;
  SampleTime adc2_sample_time = PERIOD_7;
  AdcClockDivider adc1_clock_div = NO_DIV;
  AdcClockDivider adc2_clock_div = NO_DIV;
  AdcResolution adc1_resolution = BIT_12;
  AdcResolution adc2_resolution = BIT_12;
};

struct AdcTrigRes {
  // get result of conversion for trigger T in chain segment S
  Voltage trig_res(AdcEtcTrigger trig, int segment);
};

namespace adc_etc {
  extern int TRIG0_SIGNAL_SINK;
  extern int TRIG1_SIGNAL_SINK;

  void begin(const AdcEtcBeginInfo& beginInfo);
  Voltage read_single(ain_pin pin);
}
/* struct AdcEtc { */
/*   static bool adc_initialized; */
/*   static void begin(AdcEtcBeginInfo &info); */
/*   static uint16_t readSingle(int pin); */
/*  */
/* }; */
