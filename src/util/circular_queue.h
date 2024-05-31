#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

template <typename T, size_t SIZE> struct CircularQueue {
public:
  CircularQueue() : m_front(-1), m_back(-1) {}

  bool enqueue(const T &v) {
    if ((m_front == 0 && m_back == SIZE - 1) ||
        ((m_back + 1) % SIZE == m_front)) {
      return false;
    }
    if (m_front == -1) {
      m_front = 0;
      m_back = 0;
      m_buffer[m_back].value = v;
      return true;
    }
    if (m_back == SIZE - 1 && m_front != 0) {
      m_back = 0;
      m_buffer[m_back].value = v;
      return true;
    }
    m_buffer[++m_back].value = v;
    return true;
  }

  std::optional<T> dequeue() {
    if (m_front == -1) {
      return std::nullopt;
    }
    if (m_front == m_back) {
      size_t i = m_front;
      m_front = -1;
      m_back = -1;
      return m_buffer[i].value;
    }
    if (m_front == SIZE - 1) {
      size_t i = m_front;
      m_front = 0;
      return m_buffer[i].value;
    }
    return m_buffer[m_front++].value;
  }


private:
  union Elem {
    T value;
    uint8_t _bytes;
  };
  size_t m_back;
  size_t m_front;
  std::array<Elem, SIZE> m_buffer;
};
