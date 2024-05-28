#include "xbar.h"

#include <cinttypes>
#include <imxrt.h>


void xbar::connect(unsigned int source, unsigned int sink)
// from SciMo
// from https://github.com/manitou48/teensy4/blob/master/pitxbaradc.ino
{
  if (source >= 88)
    return;
  if (sink >= 132)
    return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (sink / 2);
  uint16_t val = *xbar;
  if (!(sink & 1)) {
    val = (val & 0xFF00) | source;
  } else {
    val = (val & 0x00FF) | (source << 8);
  }
  *xbar = val;
}

void xbar::begin() {
  static bool init = false;
  if (init) {
    return;
  }
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  init = true;
  /* // PWM -> ADC_ETC_TRIG0 */
  /* xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00); */
  /* // TODO PWM -> ADC_ETC_TRIG1 (half period trigger) */
  /* xbar::connect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG1, XBARA1_OUT_ADC_ETC_TRIG01); */
}
