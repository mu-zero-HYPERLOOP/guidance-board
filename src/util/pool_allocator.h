#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <avr/pgmspace.h>

template <size_t ELEMENT_SIZE, size_t CAPACITY>
struct PoolAllocator {

  PoolAllocator() {
    for (size_t i = 1; i < CAPACITY; ++i) {
      m_pool[i - 1].m_next = m_pool.data() + i;
    }
    m_pool[CAPACITY - 1].m_next = nullptr;
    m_freelist = &m_pool[0];
  }

  template <typename T> T *alloc() {
    static_assert(sizeof(T) == ELEMENT_SIZE);
    if (m_freelist == nullptr) {
      return nullptr;
    }
    T *memory = reinterpret_cast<T *>(&m_freelist->m_element);
    m_freelist = m_freelist->m_next;
    return memory;
  }

  template <typename T> void free(T *memory) {
    static_assert(sizeof(T) <= ELEMENT_SIZE);
    assert(memory >= m_pool[0] && memory <= m_pool[CAPACITY - 1]);
    Entry *entry = reinterpret_cast<Entry *>(memory);
    entry->m_next = m_freelist;
    m_freelist = entry;
  }

private:
  union Entry {
    uint8_t m_element[ELEMENT_SIZE];
    union Entry *m_next;
  };

private:
  std::array<Entry, CAPACITY> m_pool;
  Entry *m_freelist;
};
