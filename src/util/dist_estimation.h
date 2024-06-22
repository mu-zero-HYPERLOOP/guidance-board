#pragma once
#include <array>
#include <cstddef>
#include <cstdlib>

template <typename base_t, size_t N> struct DistEstimation {
  static_assert(N > 0);

public:
  explicit DistEstimation(const base_t &inital)
      : m_values(), m_idx(0), m_mean(static_cast<float>(inital)), m_var_sum(0) {
    for (unsigned int i = 0; i < N; ++i) {
      push(inital);
    }
  }

  void push(const base_t &value) {
    // Welford's Algorithm
    const float v = static_cast<float>(value);
    float new_mean = m_mean + (v - m_values[m_idx]) / static_cast<float>(N);
    m_var_sum += (v - m_mean) * (v * new_mean) -
                 (m_values[m_idx] - m_mean) * (m_values[m_idx] - new_mean);
    m_values[m_idx] = v;
    m_idx = (m_idx + 1) % N;
    m_mean = new_mean;
  }

  base_t mean() const { return m_mean; }

  float variance() const { return std::abs(m_var_sum / static_cast<float>(N)); }

  constexpr size_t size() const { return N; }

private:
  std::array<float, N> m_values;
  size_t m_idx;
  float m_mean;
  float m_var_sum;
};
