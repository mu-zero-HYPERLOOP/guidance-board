#pragma once

#include "firmware/guidance_board.h"
#include "print.h"
#include "util/circular_queue.h"
#include "util/interval.h"
#include <array>
#include <cmath>

template <size_t COL> struct CsvWriter {

  void print_header(std::array<const char *, COL> headers) {
    bool first = true;
    for (size_t i = 0; i < headers.size(); ++i) {
      if (!first) {
        debugPrintChar(',');
      }
      first = false;
      debugPrintf("%s", headers[i]);
    }
    debugPrintChar('\n');
  }

  constexpr size_t columns() { return COL; }

  void push(std::array<float, COL> row) {
    auto _lck = guidance_board::InterruptLock::acquire();
    m_buffer.enqueue(row);
  }

  void push_from_isr(const std::array<float, COL> &row) {
    m_buffer.enqueue(row);
  }

  /// tries to print all new content to the console.
  void consume() {
    for (size_t i = 0; i < 100; ++i) {
      std::optional<std::array<float, COL>> row;
      {
        auto _lck = guidance_board::InterruptLock::acquire();
        row = m_buffer.dequeue();
      }
      if (!row.has_value()) {
        break;
      }
      bool first = true;
      for (size_t i = 0; i < row->size(); ++i) {
        if (!first) {
          debugPrintChar(',');
        }
        first = false;
        debugPrintf("%s", (*row)[i]);
      }
      debugPrintChar('\n');
    }
    if (flush_interval.next()) {
      debugPrintFlush();
    }
  }

private:
  Interval flush_interval{10_Hz};
  CircularQueue<std::array<float, COL>, 10000> m_buffer;
};
