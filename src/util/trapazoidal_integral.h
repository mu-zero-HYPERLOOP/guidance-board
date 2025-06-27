#pragma once

#include "util/metrics.h"
#include <algorithm>
class TrapazoidalIntegral{
public:
  explicit TrapazoidalIntegral(float inital)
      : m_integral(static_cast<float>(inital)), m_prev_val(0.0f) {}

  void reset(float value) {
    m_integral = static_cast<float>(value);
    m_prev_val = 0.0f;
  }

  inline void integrate(float v, Time dt) {
    const float f = static_cast<float>(v);
    m_integral += (f + m_prev_val) * 0.5 * static_cast<float>(dt);
    m_prev_val = f;
  }

  inline void clamp(float min, float max) {
    m_integral = std::clamp(m_integral, min, max);
  }

  inline float get() { return m_integral; }

private:
  float m_integral;
  float m_prev_val;
};
