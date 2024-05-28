#pragma once

template<typename base_t>
class ExponentialMovingAverage {
public:
  ExponentialMovingAverage(float alpha, base_t inital = 0)
      : m_alpha(alpha), m_one_minus_alpha(1.0 - m_alpha), m_val(inital) {}

  void push(base_t x) { m_val = x * m_alpha + (m_one_minus_alpha)*m_val; }

  void push(base_t x) volatile { m_val = x * m_alpha + (m_one_minus_alpha)*m_val; }

  base_t get() { return m_val; }
  base_t get() volatile{ return m_val; }

private:
  float m_alpha;
  float m_one_minus_alpha;
  base_t m_val;
};
