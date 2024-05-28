#pragma once

#include "util/metrics.h"

namespace sensors::formula {

static constexpr inline Distance displacement420(Current i) {
  return (i - 4_mA) * (50_mm - 20_mm) / 16_mA + 20_mm;
}

/* static constexpr Current inv_displacement420(Distance d) { */
/*   return Current((d + 4_mm) / 1_m); */
/* } */

} // namespace sensors::formula
