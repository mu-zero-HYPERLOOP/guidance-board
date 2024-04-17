#pragma once

#include "metrics.h"
#include <chrono>
#include <cstdint>

class Timestamp {
  friend Timestamp operator+(const Timestamp &, const class Duration &);
  friend Timestamp operator+(const volatile Timestamp &,
                             const volatile class Duration &);
  friend Timestamp operator+(const Timestamp &,
                             const volatile class Duration &);
  friend Timestamp operator+(const volatile Timestamp &,
                             const class Duration &);

  friend Timestamp operator+(const Duration &, const class Timestamp &);
  friend Timestamp operator+(const volatile Duration &,
                             const volatile class Timestamp &);
  friend Timestamp operator+(const Duration &,
                             const volatile class Timestamp &);
  friend Timestamp operator+(const volatile Duration &,
                             const class Timestamp &);

public:
  inline static Timestamp now() {
    using namespace std::chrono;
    milliseconds x =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return Timestamp(x.count());
  }

  Timestamp(const Timestamp &o) : Timestamp(o.m_time_us) {}
  Timestamp(const volatile Timestamp &o) : Timestamp(o.m_time_us) {}
  Timestamp(Timestamp &&o) : Timestamp(o.m_time_us) {}
  Timestamp(volatile Timestamp &o) : Timestamp(o.m_time_us) {}

  Timestamp &operator=(const Timestamp &o) {
    m_time_us = o.m_time_us;
    return *this;
  }
  volatile Timestamp &operator=(const volatile Timestamp &o) volatile {
    m_time_us = o.m_time_us;
    return *this;
  }
  Timestamp &operator=(const volatile Timestamp &o) {
    m_time_us = o.m_time_us;
    return *this;
  }
  volatile Timestamp &operator=(const Timestamp &o) volatile {
    m_time_us = o.m_time_us;
    return *this;
  }

  Timestamp &operator=(Timestamp &&o) {
    m_time_us = o.m_time_us;
    return *this;
  }
  volatile Timestamp &operator=(volatile Timestamp &&o) volatile {
    m_time_us = o.m_time_us;
    return *this;
  }
  Timestamp &operator=(volatile Timestamp &&o) {
    m_time_us = o.m_time_us;
    return *this;
  }
  volatile Timestamp &operator=(Timestamp &&o) volatile {
    m_time_us = o.m_time_us;
    return *this;
  }

  explicit inline operator uint64_t() const { return m_time_us; }
  explicit inline operator uint64_t() const volatile { return m_time_us; }

private:
  explicit Timestamp(uint64_t time) : m_time_us(time) {}
  uint64_t m_time_us;
};

class Duration {
  friend Duration operator-(const Timestamp &, const Timestamp &);
  friend Duration operator-(const volatile Timestamp &,
                            const volatile Timestamp &);
  friend Duration operator-(const Timestamp &, const volatile Timestamp &);
  friend Duration operator-(const volatile Timestamp &, const Timestamp &);

  friend Duration operator+(const Duration &, const Duration &);
  friend Duration operator+(const volatile Duration &,
                            const volatile Duration &);
  friend Duration operator+(const Duration &, const volatile Duration &);
  friend Duration operator+(const volatile Duration &, const Duration &);

  friend Duration operator-(const Duration &, const Duration &);
  friend Duration operator-(const volatile Duration &,
                            const volatile Duration &);
  friend Duration operator-(const Duration &, const volatile Duration &);
  friend Duration operator-(const volatile Duration &, const Duration &);

public:
  constexpr Duration() : m_us(0) {}
  constexpr Duration(Time x)
      : m_us(static_cast<uint64_t>(static_cast<float>(x) * 1e6)) {}

  explicit operator uint64_t() const { return m_us; }
  explicit operator uint64_t() const volatile { return m_us; }

  explicit operator Time() const { return Time(m_us / 1e6); }
  explicit operator Time() const volatile { return Time(m_us / 1e6); }

  inline uint64_t as_us() const { return m_us; }
  inline uint64_t as_us() const volatile { return m_us; }
  inline uint64_t as_ms() const { return m_us / 1000; }
  inline uint64_t as_ms() const volatile { return m_us / 1000; }

  inline Duration &operator+=(const Duration &o) {
    m_us += o.m_us;
    return *this;
  }
  inline volatile Duration &operator+=(const volatile Duration &o) volatile {
    m_us += o.m_us;
    return *this;
  }
  inline Duration &operator+=(const volatile Duration &o) {
    m_us += o.m_us;
    return *this;
  }
  inline volatile Duration &operator+=(const Duration &o) volatile {
    m_us += o.m_us;
    return *this;
  }
  inline Duration &operator-=(const Duration &o) {
    m_us -= o.m_us;
    return *this;
  }
  inline volatile Duration &operator-=(const volatile Duration &o) volatile {
    m_us -= o.m_us;
    return *this;
  }
  inline Duration &operator-=(const volatile Duration &o) {
    m_us -= o.m_us;
    return *this;
  }
  inline volatile Duration &operator-=(const Duration &o) volatile {
    m_us -= o.m_us;
    return *this;
  }

private:
  explicit Duration(uint64_t us) : m_us(us) {}
  uint64_t m_us;
};

inline Duration operator-(const Timestamp &a, const Timestamp &b) {
  return Duration(static_cast<uint64_t>(a) - static_cast<uint64_t>(b));
}
inline Duration operator-(const volatile Timestamp &a,
                          const volatile Timestamp &b) {
  return Duration(static_cast<uint64_t>(a) - static_cast<uint64_t>(b));
}
inline Duration operator-(const Timestamp &a, const volatile Timestamp &b) {
  return Duration(static_cast<uint64_t>(a) - static_cast<uint64_t>(b));
}
inline Duration operator-(const volatile Timestamp &a, const Timestamp &b) {
  return Duration(static_cast<uint64_t>(a) - static_cast<uint64_t>(b));
}

inline Timestamp operator+(const Timestamp &a, const Duration &b) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Timestamp operator+(const volatile Timestamp &a,
                           const volatile Duration &b) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Timestamp operator+(const Timestamp &a, const volatile Duration &b) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Timestamp operator+(const volatile Timestamp &a, const Duration &b) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}

inline Timestamp operator+(const Duration &b, const Timestamp &a) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Timestamp operator+(const volatile Duration &b,
                           const volatile Timestamp &a) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Timestamp operator+(const Duration &b, const volatile Timestamp &a) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Timestamp operator+(const volatile Duration &b, const Timestamp &a) {
  return Timestamp(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}

inline Duration operator+(const Duration &a, const Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Duration operator+(const volatile Duration &a,
                          const volatile Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Duration operator+(const Duration &a, const volatile Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Duration operator+(const volatile Duration &a, const Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}

inline Duration operator-(const Duration &a, const Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Duration operator-(const volatile Duration &a,
                          const volatile Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Duration operator-(const Duration &a, const volatile Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}
inline Duration operator-(const volatile Duration &a, const Duration &b) {
  return Duration(static_cast<uint64_t>(a) + static_cast<uint64_t>(b));
}

static inline bool operator<(const Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) < static_cast<uint64_t>(b);
}
static inline bool operator<(const volatile Duration &a,
                             const volatile Duration &b) {
  return static_cast<uint64_t>(a) < static_cast<uint64_t>(b);
}
static inline bool operator<(const Duration &a, const volatile Duration &b) {
  return static_cast<uint64_t>(a) < static_cast<uint64_t>(b);
}
static inline bool operator<(const volatile Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) < static_cast<uint64_t>(b);
}

static inline bool operator>(const Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) > static_cast<uint64_t>(b);
}
static inline bool operator>(const volatile Duration &a,
                             const volatile Duration &b) {
  return static_cast<uint64_t>(a) > static_cast<uint64_t>(b);
}
static inline bool operator>(const Duration &a, const volatile Duration &b) {
  return static_cast<uint64_t>(a) > static_cast<uint64_t>(b);
}
static inline bool operator>(const volatile Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) > static_cast<uint64_t>(b);
}

static inline bool operator<=(const Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) <= static_cast<uint64_t>(b);
}
static inline bool operator<=(const volatile Duration &a,
                              const volatile Duration &b) {
  return static_cast<uint64_t>(a) <= static_cast<uint64_t>(b);
}
static inline bool operator<=(const Duration &a, const volatile Duration &b) {
  return static_cast<uint64_t>(a) <= static_cast<uint64_t>(b);
}
static inline bool operator<=(const volatile Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) <= static_cast<uint64_t>(b);
}

static inline bool operator>=(const Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) >= static_cast<uint64_t>(b);
}
static inline bool operator>=(const volatile Duration &a,
                              const volatile Duration &b) {
  return static_cast<uint64_t>(a) >= static_cast<uint64_t>(b);
}
static inline bool operator>=(const Duration &a, const volatile Duration &b) {
  return static_cast<uint64_t>(a) >= static_cast<uint64_t>(b);
}
static inline bool operator>=(const volatile Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) >= static_cast<uint64_t>(b);
}

static inline bool operator==(const Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}
static inline bool operator==(const volatile Duration &a,
                              const volatile Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}
static inline bool operator==(const Duration &a, const volatile Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}
static inline bool operator==(const volatile Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}

static inline bool operator!=(const Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}
static inline bool operator!=(const volatile Duration &a,
                              const volatile Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}
static inline bool operator!=(const Duration &a, const volatile Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}
static inline bool operator!=(const volatile Duration &a, const Duration &b) {
  return static_cast<uint64_t>(a) != static_cast<uint64_t>(b);
}