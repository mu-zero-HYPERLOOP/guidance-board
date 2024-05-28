#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <optional>
#include <tuple>

template <typename Key, typename Value, size_t CAP> struct MinHeap {

private:
  struct Entry {
    Key m_key;
    Value m_value;
  };

public:
  struct Iterator;
  void decrease_key(size_t position, const Key &key) {
    if (position == 0) {
      return;
    }
    assert(position < m_heap_size);
    assert(m_heap[position].m_key >= key);
    m_heap[position].m_key = key;
    size_t parent = (position - 1) / 2;
    while (m_heap[parent].m_key > m_heap[position].m_key) {
      // swap parent and position
      Entry tmp = m_heap[parent];
      m_heap[parent] = m_heap[position];
      m_heap[position] = tmp;
      position = parent;
      parent = (position - 1) / 2;
    }
  }

  void increase_key(size_t position, const Key &key) {
    if (position == m_heap_size) {
      return;
    }
    assert(position < m_heap_size);
    assert(m_heap[position].m_key <= key);

    Entry job = m_heap[position];
    job.m_key = key;
    
    size_t hsize = m_heap_size / 2;
    while (position < hsize) {
      size_t left_child = position * 2 + 1;
      size_t right_child = left_child + 1;
      size_t min_child;
      if (right_child < m_heap_size &&
          m_heap[left_child].m_key >= m_heap[right_child].m_key) {
        min_child = right_child;
      } else {
        min_child = left_child;
      }
      if (key <= m_heap[min_child].m_key) {
        break;
      }
      m_heap[position] = m_heap[min_child];
      position = min_child;
    }
    m_heap[position] = job;
  }

  bool insert(const Key &key, const Value &value) {
    if (m_heap_size >= CAP) {
      return false;
    }
    m_heap[m_heap_size].m_key = key;
    m_heap[m_heap_size].m_value = value;
    m_heap_size += 1; 
    decrease_key(m_heap_size - 1, key);
    return true;
  }

  std::optional<const Value*> min_value() {
    if (m_heap_size == 0) {
      return std::nullopt;
    } else {
      return &m_heap[0].m_value;
    }
  }

  std::optional<std::tuple<const Key*, const Value*>> min() {
    if (m_heap_size == 0) {
      return std::nullopt;
    } else {
      return std::make_pair(&m_heap[0].m_key, &m_heap[0].m_value);
    }
  }

  void pop() {
    assert(m_heap_size > 0);
    m_heap[0] = m_heap[m_heap_size - 1];
    m_heap_size -= 1;
    increase_key(0, m_heap[0].m_key);
  }

  size_t size() { return m_heap_size; }

  Iterator begin() { return Iterator(m_heap.data()); }

  Iterator end() { return Iterator(m_heap.data() + m_heap_size); }

  struct Iterator {
  public:
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = Value;
    using pointer = Value *;
    using reference = Value &;

    Iterator(Entry *ptr) : m_ptr(ptr) {}

    reference operator*() const { return m_ptr->m_value; }
    pointer operator->() { return &m_ptr->m_value; }

    Iterator &operator++() {
      ++m_ptr;
      return *this;
    }

    Iterator &operator--() {
      --m_ptr;
      return *this;
    }

    Iterator &operator++(int) {
      Iterator tmp = *this;
      ++m_ptr;
      return tmp;
    }

    Iterator &operator--(int) {
      Iterator tmp = *this;
      --m_ptr;
      return tmp;
    }

    friend bool operator==(const Iterator &a, const Iterator &b) {
      return a.m_ptr == b.m_ptr;
    };

    friend bool operator!=(const Iterator &a, const Iterator &b) {
      return a.m_ptr != b.m_ptr;
    };

    friend difference_type operator-(const Iterator&a, const Iterator& b) {
      return a.m_ptr - b.m_ptr;
    }

  private:
    Entry *m_ptr;
  };

  std::array<Entry, CAP> m_heap;
  size_t m_heap_size;
};
