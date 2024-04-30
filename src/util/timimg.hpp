#pragma once

#include "util/ema.hpp"
#include "util/metrics.hpp"
#include "util/timestamp.hpp"

struct Timing {
  inline Timing() : m_start(Timestamp::now()) {}
  inline auto start() -> void { m_start = Timestamp::now(); }
  inline auto time() -> Duration { return Timestamp::now() - m_start; }

private:
  Timestamp m_start;
};

struct IntervalTiming {
  inline IntervalTiming() : m_last(Timestamp::now()), m_ema(0.001, 0_s) {}

  auto tick() -> void {
    const auto now = Timestamp::now();
    Duration time_since_last_tick = now - m_last;
    m_last = now;
    /* Serial.printf("time_since = %f\n",
     * static_cast<float>(time_since_last_tick.as_us()) / 1.e6); */
    m_ema.push(static_cast<Time>(
        static_cast<float>(time_since_last_tick.as_us()) / 1e6));
  }

  auto frequency() -> Frequency { return 1.0f / period(); };

  auto period() -> Time { return m_ema.get(); };

private:
  Timestamp m_last;
  ExponentialMovingAverage<Time> m_ema;
};
