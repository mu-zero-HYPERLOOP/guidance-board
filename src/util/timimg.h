#pragma once

#include "util/ema.hpp"
#include "util/metrics.h"
#include "util/timestamp.h"

struct Timing {
  inline Timing() : m_start() {}
  inline auto start() -> void { m_start = Timestamp::now(); }
  inline auto time() -> Duration { return Timestamp::now() - m_start; }

private:
  Timestamp m_start;
};

struct IntervalTiming {
  inline IntervalTiming(float alpha = 0.5) : m_last(Timestamp::now()), m_ema(alpha, 0_s) {}

  auto tick() -> void {
    const auto now = Timestamp::now();
    Duration time_since_last_tick = now - m_last;
    m_last = now;
    m_ema.push(static_cast<Time>(static_cast<float>(time_since_last_tick.as_us()) / 1e6));
  }

  auto tick() volatile -> void {
    const auto now = Timestamp::now();
    Duration time_since_last_tick = now - m_last;
    m_last = now;
    m_ema.push(static_cast<Time>(static_cast<float>(time_since_last_tick.as_us()) / 1e6));
  }


  auto frequency() -> Frequency { return 1.0f / m_ema.get(); };

  auto period() -> Time { return m_ema.get(); };

  auto frequency() volatile -> Frequency { return 1.0f / m_ema.get(); };

  auto period() volatile -> Time { return m_ema.get(); };

private:
  Timestamp m_last;
  ExponentialMovingAverage<Time> m_ema;
};
