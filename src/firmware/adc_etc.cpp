#include "adc_etc.h"
#include "analog.c"
#include "firmware/guidance_board.h"
#include "imxrt.h"
#include "print.h"
#include <Arduino.h>

#define PRREG(x)                                                               \
  Serial.print(#x " 0x");                                                      \
  Serial.println(x, HEX)

static volatile uint8_t done0_triggers = 0;
static volatile uint8_t done1_triggers = 0;
static volatile uint8_t done2_triggers = 0;

static bool adc_etc_initalized = false;
/*  */
/* bool adc_etc::adc_initialized = false; */

int adc_etc::TRIG0_SIGNAL_SINK = XBARA1_OUT_ADC_ETC_TRIG00;
int adc_etc::TRIG1_SIGNAL_SINK = XBARA1_OUT_ADC_ETC_TRIG01;

__attribute__((weak)) void adc_etc_done0_isr(AdcTrigRes res) {
  Serial.println("ADC ETC DONE0 interrupt triggered:");
}
void adc_etc_0_isr() {
  ADC_ETC_DONE0_1_IRQ |= done0_triggers;
  AdcTrigRes res;
  adc_etc_done0_isr(res);
  asm("dsb");
}

__attribute__((weak)) void adc_etc_done1_isr(AdcTrigRes res) {
  Serial.println("ADC ETC DONE1 interrupt triggered:");
}
void adc_etc_1_isr() {
  ADC_ETC_DONE0_1_IRQ |= done1_triggers << 16;
  AdcTrigRes res;
  adc_etc_done1_isr(res);
  asm("dsb");
}

__attribute__((weak)) void adc_etc_done2_isr(AdcTrigRes res) {
  Serial.println("ADC ETC DONE2 interrupt triggered:");
}

volatile bool sw_trig_coco;

void adc_etc_2_isr() {
  ADC_ETC_DONE2_ERR_IRQ |= done2_triggers;
  sw_trig_coco = true;

  asm("dsb");
}

static float max_adc1_value = 0;
static float max_adc2_value = 0;

void adc_etc::begin(const AdcEtcBeginInfo &info) {
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

  ADC1_CFG =
      ADC_CFG_OVWREN; // allows to overwrite unread result from conversion
                      // seems to be necessary for etc trigger to work
  ADC1_CFG |= ADC_CFG_AVGS(info.adc1_avg); // number of hardware averages taken
  ADC1_CFG |= ADC_CFG_ADTRG; // enable hardware trigger for conversion
  if (info.adc1_high_speed)
    ADC1_CFG |= ADC_CFG_ADHSC; // enable high speed conversion
  ADC1_CFG |= ADC_CFG_ADSTS(info.adc1_sample_time &
                            0b11); // number of clock cycles for sampling
  ADC1_CFG |= ADC_CFG_ADIV(info.adc1_clock_div); // clock divider
  if (info.adc1_sample_time >> 2)
    ADC1_CFG |= ADC_CFG_ADLSMP; // enable long sampling time (see ADSTS)
  ADC1_CFG |=
      ADC_CFG_MODE(info.adc1_resolution); // conversion resolution. 0b10 = 12bit
  ADC1_CFG |= ADC_CFG_ADICLK(0b00);       // input clock select
  ADC1_HC0 =
      ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC
  ADC1_HC1 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC2 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC3 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC4 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC5 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC6 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC1_HC7 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC

  ADC2_CFG =
      ADC_CFG_OVWREN; // allows to overwrite unread result from conversion
                      // seems to be necessary for etc trigger to work
  ADC2_CFG |= ADC_CFG_AVGS(info.adc2_avg); // number of hardware averages taken
  ADC2_CFG |= ADC_CFG_ADTRG; // enable hardware trigger for conversion
  if (info.adc2_high_speed)
    ADC2_CFG |= ADC_CFG_ADHSC; // enable high speed conversion
  ADC2_CFG |= ADC_CFG_ADSTS(info.adc2_sample_time &
                            0b11); // number of clock cycles for sampling
  ADC2_CFG |= ADC_CFG_ADIV(info.adc2_clock_div); // clock divider
  if (info.adc2_sample_time >> 2)
    ADC2_CFG |= ADC_CFG_ADLSMP; // enable long sampling time (see ADSTS)
  ADC2_CFG |=
      ADC_CFG_MODE(info.adc2_resolution); // conversion resolution. 0b10 = 12bit
  ADC2_CFG |= ADC_CFG_ADICLK(0b00);       // input clock select
  ADC2_HC0 =
      ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC
  ADC2_HC1 =
      ADC_HC_ADCH(16); // channel of hardware trigger 1 is controlled by ADC_ETC
  ADC2_HC2 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC3 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC4 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC5 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC6 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC
  ADC2_HC7 =
      ADC_HC_ADCH(16); // channel of hardware trigger is controlled by ADC_ETC

  ADC_ETC_CTRL = ADC_ETC_CTRL_SOFTRST;
  ADC_ETC_CTRL &= ~ADC_ETC_CTRL_SOFTRST;
  // trigger soft reset of all registers in ADC_ETC
  ADC_ETC_CTRL &= ~ADC_ETC_CTRL_TSC_BYPASS;
  // TSC bypass can not be used together with SyncMode
  for (int chain = 0; chain < info.num_chains; chain++) {
    int trig_num = static_cast<int>(info.chains[chain].trig_num);
    ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(1 << trig_num);

    // ETC_TRIG control register:
    IMXRT_ADC_ETC.TRIG[trig_num].CTRL = ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(
        info.chains[chain].chain_priority & 0b111);
    // set chain priority
    if (info.chains[chain].trig_sync)
      IMXRT_ADC_ETC.TRIG[trig_num].CTRL |= ADC_ETC_TRIG_CTRL_SYNC_MODE;
    // activate SyncMode
    IMXRT_ADC_ETC.TRIG[trig_num].CTRL |= ADC_ETC_TRIG_CTRL_TRIG_CHAIN(
        (info.chains[chain].chain_length - 1) & 0b111);
    // set chain length
    if (info.chains[chain].software_trig) {
      IMXRT_ADC_ETC.TRIG[trig_num].CTRL |= ADC_ETC_TRIG_CTRL_SW_TRIG;
      // enable software trigger instead of hardware trigger
    }

    // ETC_TRIG chain registers:
    for (int segm = 0; segm < info.chains[chain].chain_length; segm++) {
      *((&(IMXRT_ADC_ETC.TRIG[trig_num].CHAIN_1_0)) + segm / 2) = 0;
      // reset trigger chain config
    }
    for (int segm = 0; segm < info.chains[chain].chain_length; segm++) {
      volatile uint32_t *chain_base =
          (&(IMXRT_ADC_ETC.TRIG[trig_num].CHAIN_1_0)) + (segm / 2);
      if (segm % 2) {
        *chain_base |=
            ADC_ETC_TRIG_CHAIN_IE1(segm == info.chains[chain].chain_length - 1
                                       ? info.chains[chain].intr
                                       : DoneInterrupt::NONE);
        // enable done interrupt only for last segment
        *chain_base |=
            ADC_ETC_TRIG_CHAIN_CSEL1(pin_to_channel[static_cast<uint8_t>(
                info.chains[chain].read_pins[segm])]);
        // select read channel
        // trigger 0-3 read from adc1, trigger 4-7 read from adc2
      } else {
        *chain_base |=
            ADC_ETC_TRIG_CHAIN_IE0(segm == info.chains[chain].chain_length - 1
                                       ? info.chains[chain].intr
                                       : DoneInterrupt::NONE);
        // enable done interrupt only for last segment
        *chain_base |=
            ADC_ETC_TRIG_CHAIN_CSEL0(pin_to_channel[static_cast<uint8_t>(
                info.chains[chain].read_pins[segm])]);
        // select read channel
      }
      *chain_base |= ADC_ETC_TRIG_CHAIN_B2B1 | ADC_ETC_TRIG_CHAIN_B2B0;
      // trigger next segments conversion immediatedly after segment finished
      *chain_base |= ADC_ETC_TRIG_CHAIN_HWTS1(1 << (trig_num)) |
                     ADC_ETC_TRIG_CHAIN_HWTS0(1 << (trig_num));
      // select harware trigger for ADC
      // trigger 0-3 read from adc1, trigger 4-7 read from adc2
    }
    switch (info.chains[chain].intr) {
    case DoneInterrupt::DONE0:
      done0_triggers |= 1 << static_cast<uint8_t>(info.chains[chain].trig_num);
      attachInterruptVector(IRQ_ADC_ETC0, adc_etc_0_isr);
      NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
      break;
    case DoneInterrupt::DONE1:
      done1_triggers |= 1 << static_cast<uint8_t>(info.chains[chain].trig_num);
      attachInterruptVector(IRQ_ADC_ETC1, adc_etc_1_isr);
      NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
      break;
    case DoneInterrupt::DONE2:
      break;
    default:
      break;
    }
  }

  // single reads
  ADC1_HC3 = ADC_HC_ADCH(16);

  ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(1 << 3);

  ADC_ETC_TRIG3_CTRL = ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(0); // lowest priority
  ADC_ETC_TRIG3_CTRL |= ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0);   // chain length of 1
  ADC_ETC_TRIG3_CTRL |= ADC_ETC_TRIG_CTRL_TRIG_MODE; // enable software trigger

  ADC_ETC_TRIG3_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_IE0(0b11); // no done interrupt
  ADC_ETC_TRIG3_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_B2B0;
  ADC_ETC_TRIG3_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_HWTS0(1 << 3);

  ADC2_HC7 = ADC_HC_ADCH(16);

  ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(1 << 7);

  ADC_ETC_TRIG7_CTRL = ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(0); // lowest priority
  ADC_ETC_TRIG7_CTRL |=
      ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0);               // 0 -> chain length of 1
  ADC_ETC_TRIG7_CTRL |= ADC_ETC_TRIG_CTRL_TRIG_MODE; // enable software trigger

  ADC_ETC_TRIG7_CHAIN_1_0 = ADC_ETC_TRIG_CHAIN_IE0(0b11); // no done interrupt
  ADC_ETC_TRIG7_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_B2B0;
  ADC_ETC_TRIG7_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_HWTS0(1 << 7);

  done2_triggers |= 1 << static_cast<uint8_t>(3);
  done2_triggers |= 1 << static_cast<uint8_t>(7);
  attachInterruptVector(IRQ_ADC_ETC2, adc_etc_2_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC2);

  adc_etc_initalized = true;
}

// uses etc trigger 3 and 7 with software trigger.
// trigger 3 -> uses adc1, trigger 7 -> uses adc2
// Dont use this function if trigger 3 or 7 is used for regular etc chain.
Voltage adc_etc::read_single(ain_pin pin) {
  if (!adc_etc_initalized) {
    Serial.println("adc not initlialized");
    return 0_V;
  }
  // get channel and the ADC module connected to it.
  // 0x7f & ch is the channel number
  // 0x80 & ch is the adc module : 0 -> ADC1, 1 -> ADC2.
  uint8_t ch = pin_to_channel[static_cast<uint8_t>(pin)];
  if (!(ch & 0x80)) { // use adc1

    ADC_ETC_TRIG3_CHAIN_1_0 &= ~0xF;
    ADC_ETC_TRIG3_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_CSEL0(ch & 0x7f);

    sw_trig_coco = false;
    ADC_ETC_TRIG3_CTRL |= ADC_ETC_TRIG_CTRL_SW_TRIG; // trigger a conversion
    while (!sw_trig_coco); // poll until coco.
    return (ADC_ETC_TRIG3_RESULT_1_0 & 0xfff) * 3.3_V / max_adc1_value;
  } else { // use adc2
    ADC_ETC_TRIG7_CHAIN_1_0 &= ~0xF;
    ADC_ETC_TRIG7_CHAIN_1_0 |= ADC_ETC_TRIG_CHAIN_CSEL0(ch & 0x7f);

    sw_trig_coco = false;
    ADC_ETC_TRIG7_CTRL |= ADC_ETC_TRIG_CTRL_SW_TRIG; // trigger a conversion
    while (!sw_trig_coco); // poll until coco.
    return (ADC_ETC_TRIG7_RESULT_1_0 & 0xfff) * 3.3_V / max_adc2_value;
  }
}

Voltage AdcTrigRes::trig_res(AdcEtcTrigger trig, int segment) {
  float max_value = trig < 4 ? max_adc1_value : max_adc2_value;
  volatile uint32_t *result_base =
      &IMXRT_ADC_ETC.TRIG[trig].RESULT_1_0 + (segment / 2);
  if (segment % 2) {
    return ((*result_base >> 16) & 0xfff) * 3.3_V / max_value;
  } else {
    return (*result_base & 0xfff) * 3.3_V / max_value;
  }
}
