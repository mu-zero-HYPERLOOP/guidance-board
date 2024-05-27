#pragma once

#include "util/ema.h"
#include "util/metrics.h"
#include "util/timestamp.h"

struct Timing {
  inline Timing() : m_start(Timestamp::now()) {}
  inline auto start() -> void { m_start = Timestamp::now(); }
  inline auto start() volatile -> void { m_start = Timestamp::now(); }
  inline auto time() -> Duration { return Timestamp::now() - m_start; }
  inline auto time() volatile const -> Duration { return Timestamp::now() - m_start; }

private:
  Timestamp m_start;
};

struct IntervalTiming {
  inline IntervalTiming() : m_last(Timestamp::now()), m_ema(0.1, 0_s) {}

  auto tick() -> void {
    const auto now = Timestamp::now();
    Duration time_since_last_tick = now - m_last;
    m_last = now;
    /* Serial.printf("time_since = %f\n",
     * static_cast<float>(time_since_last_tick.as_us()) / 1.e6); */
    m_ema.push(Time(
        static_cast<float>(time_since_last_tick.as_us()) / 1e6));
  }

  auto tick() volatile -> void {
    const auto now = Timestamp::now();
    Duration time_since_last_tick = now - m_last;
    m_last = now;
    /* Serial.printf("time_since = %f\n",
     * static_cast<float>(time_since_last_tick.as_us()) / 1.e6); */
    m_ema.push(Time(
        static_cast<float>(time_since_last_tick.as_us()) / 1e6));
  }

  auto frequency() const -> Frequency { return 1.0f / period(); };
  auto frequency() volatile const -> Frequency { return 1.0f / period(); };

  auto period() const -> Time { return m_ema.get(); };
  auto period() volatile const -> Time { return m_ema.get(); };

private:
  Timestamp m_last;
  ExponentialMovingAverage<Time> m_ema;
};
