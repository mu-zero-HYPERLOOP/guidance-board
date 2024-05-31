#pragma once

#include "util/timestamp.h"
#include <algorithm>
class PIDController {
public:
  explicit PIDController(float Kp, float Ki, float Kd, float integral_min,
                         float integral_max)
      : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_integral(0),
        m_integral_min(integral_min), m_integral_max(integral_max),
        m_prev_error() {}

  float step(float current, float target, float dt) {
    float error = target - current;

    // Trapazoidal integration.
    m_integral += (error + m_prev_error) * 0.5 * dt;

    m_integral = std::clamp(m_integral, m_integral_min, m_integral_max);
    
    float deriviate = (error - m_prev_error) / dt;

    float output = error * m_Kp + m_integral * m_Ki + deriviate * m_Kd;

    m_prev_error = error;

    return output;
  }

private:
  float m_Kp;
  float m_Ki;
  float m_Kd;

  float m_integral;
  float m_integral_min;
  float m_integral_max;
  float m_prev_error;
};
