#pragma once
#include "util/metrics.h"
#include <cmath>

struct SteinhartNtc {
  explicit constexpr SteinhartNtc(float A, float B, float C, float D)
      : m_A(A), m_B(B), m_C(C), m_D(D) {}

  Temperature inline convert(const Resistance &ntc_r) const {
    const float ln_r = log(static_cast<float>(ntc_r));
    const float ln2_r = ln_r * ln_r;
    return Temperature(1.0f /
                       (m_A + m_B * ln_r + m_C * ln2_r + m_D * ln2_r * ln_r));
  }

private:
  const float m_A;
  const float m_B;
  const float m_C;
  const float m_D;
};

struct BetaModelNtc {
public:
  explicit constexpr BetaModelNtc(float beta, Resistance r25)
      : m_beta(beta), m_r25(r25) {}

  Temperature inline apply(const Resistance &ntc_r) const {
    return Temperature(1.0f /
                       (-(log(static_cast<float>(m_r25 / ntc_r)) / m_beta) +
                        static_cast<float>((1.0f / 25_Celcius))));
  }

private:
  const float m_beta;
  const Resistance m_r25;
};
