#pragma once

#include "firmware/guidance_board.h"
#include "util/circular_queue.h"
#include "util/interval.h"
#include <array>
#include <cmath>
#include <Arduino.h>

template <size_t COL> struct CsvWriter {

  void print_header(std::array<const char *, COL> headers){
    bool first = true;
    for (size_t i = 0; i < headers.size(); ++i) {
      if (!first) {
        Serial.print(",");
      }
      first = false;
      Serial.printf("%s", headers[i]);
    }
    Serial.print("\n");
  }

  constexpr size_t columns() {
    return COL;
  }


  void push(std::array<float, COL> row) {
    auto _lck = guidance_board::InterruptLock::acquire();
    m_buffer.enqueue(row);
  }

  void push_from_isr(const std::array<float, COL>& row) {
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
          Serial.print(",");
        }
        first = false;
        Serial.printf("%f", (*row)[i]);
      }
      Serial.print("\n");
    }
    if (flush_interval.next()){
      fflush(stdout);
    }
  }

private:
  Interval flush_interval{10_Hz};
  CircularQueue<std::array<float, COL>, 10000> m_buffer;
};
