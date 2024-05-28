#pragma once

#include "util/metrics.h"
#include "util/timestamp.h"

class Timeout {
public:
  explicit Timeout(Duration timeout)
      : m_timeout(timeout), m_last_reset(Timestamp::now()) {}

  explicit Timeout(float timeout_s)
      : m_timeout(Time(timeout_s)), m_last_reset(Timestamp::now()) {}

  void reset() { m_last_reset = Timestamp::now(); }

  bool timed_out() { return Timestamp::now() - m_last_reset > m_timeout; }

  void set_timeout(const Duration &timeout) { m_timeout = timeout; }

private:
  Duration m_timeout;
  Timestamp m_last_reset;
};
