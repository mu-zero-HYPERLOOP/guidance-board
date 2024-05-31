#include "firmware/adc_etc.h"
#include <cassert>
#include "firmware/guidance_board.h"
#include <iostream>
#include "util/metrics.h"
#include <array>

static bool adc_etc_initalized = false;

int adc_etc::TRIG0_SIGNAL_SINK = 0;
int adc_etc::TRIG1_SIGNAL_SINK = 0;

__attribute__((weak)) void adc_etc_done0_isr(AdcTrigRes res) {
}

void adc_etc_0_isr() {
  AdcTrigRes res;
  adc_etc_done0_isr(res);
}

__attribute__((weak)) void adc_etc_done1_isr(AdcTrigRes res) {
}
void adc_etc_1_isr() {
  AdcTrigRes res;
  adc_etc_done1_isr(res);
}

__attribute__((weak)) void adc_etc_done2_isr(AdcTrigRes res) {
}
void adc_etc_2_isr() {
  AdcTrigRes res;
  adc_etc_done2_isr(res);
}

static float max_adc1_value = 0;
static float max_adc2_value = 0;

static std::array<std::array<ain_pin, 8>, 8> trig_mocks;

void adc_etc::begin(const AdcEtcBeginInfo &info) {
  assert(adc_etc_initalized == false);
  switch (info.adc1_resolution) {
  case BIT_8:
    max_adc1_value = 0xFF; // 255
    break;
  case BIT_10:
    max_adc1_value = 0x3FF; // 1023
  case BIT_12:
    max_adc1_value = 0xFFF; // 4095
    break;
  }
  switch (info.adc2_resolution) {
  case BIT_8:
    max_adc2_value = 0xFF;
    break;
  case BIT_10:
    max_adc2_value = 0x3FF;
    break;
  case BIT_12:
    max_adc2_value = 0xFFF;
    break;
  }

  for (size_t i = 0; i < info.num_chains; ++i) {
    const size_t trig_num = info.chains[i].trig_num;
    for (size_t j = 0; j < info.chains[i].chain_length; ++j) {
      trig_mocks[trig_num][j] = info.chains[i].read_pins[j];
    }
  }

  adc_etc_initalized = true;
}

// uses etc trigger 3 and 7 with software trigger.
// trigger 3 -> uses adc1, trigger 7 -> uses adc2
// Dont use this function if trigger 3 or 7 is used for regular etc chain.
Voltage adc_etc::read_single(ain_pin pin) {
  assert(false && "single reads are not mocked in adc etc");
  return 0_V;
}


Voltage AdcTrigRes::trig_res(AdcEtcTrigger trig, int segment) {
  return guidance_board::sync_read(trig_mocks[trig][segment]);
}
