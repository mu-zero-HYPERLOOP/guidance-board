#pragma once

#include "util/metrics.h"
namespace sensors::formula {

static constexpr float ntc_beta_from_ref(Resistance R1, Temperature T1, Resistance R2, Temperature T2) {
  return static_cast<float>(std::log(R1 / R2) / ((1.0f / T1) - 1.0f / T2));
}

static constexpr Temperature ntc_beta(Resistance R, float beta, Resistance r_ref, Temperature t_ref) {
  float inv_temp = std::log(R / r_ref) / beta + 1.0 / static_cast<float>(t_ref);
  return Temperature(1.0f / inv_temp);
}

static constexpr Resistance inv_ntc_beta(Temperature T, float beta, Resistance r_ref, 
    Temperature t_ref) {
  return Resistance(static_cast<float>(r_ref) * std::exp(
      beta * static_cast<float>(1/T - 1 / t_ref)
      ));
}

}
