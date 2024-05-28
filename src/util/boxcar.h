#pragma once
#include <array>
#include <cstddef>

/// Cool name for a simple moving average filter, where all weights are the same!
template <typename base_t, size_t N> struct BoxcarFilter {
  static_assert(N > 0);

public:
  explicit BoxcarFilter(const base_t &inital)
      : m_values(), m_sum(N * inital), m_idx(0) {
    for (unsigned int i = 0; i < N; ++i) {
      m_values[i] = inital;
    }
  }

  void push(const base_t &v) {
    m_sum -= m_values[m_idx];
    m_values[m_idx] = v;
    m_sum += m_values[m_idx];
    m_avg = m_sum / static_cast<float>(N);
    m_idx = (m_idx + 1) % N;
  }

  base_t &get() {return m_avg;}

  constexpr size_t size() {
    return N;
  }

private:
  std::array<base_t, N> m_values;
  base_t m_sum;
  size_t m_idx;
  base_t m_avg;
};

