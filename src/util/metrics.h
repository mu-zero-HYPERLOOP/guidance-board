#pragma once

#include "math.h"
#include <cmath>

template <int s, int m, int kg, int A, int K, int mol, int cd> struct Metric {
public:
  Metric() : m_value(0.0f) {}

  static constexpr Metric<0, 0, 0, 0, 1, 0, 0> from_celcius(float celcius) {
    static_assert(s == 0 && m == 0 && kg == 0 && A == 0 && K == 1 && mol == 0 && cd == 0, "only statically avaiable on Temperature");
    return Metric<0,0,0,0,1,0,0>(celcius - 273.15);
  };

  constexpr explicit Metric(const float &v) : m_value(v) {}
  constexpr explicit Metric(const volatile float &v) : m_value(v) {}

  constexpr Metric(const Metric &o) : m_value(o.m_value) {}
  constexpr Metric(const Metric volatile &o) : m_value(o.m_value) {}

  constexpr Metric(Metric &&o) : m_value(o.m_value) {}
  constexpr Metric(Metric volatile &&o) : m_value(o.m_value) {}

  float as_celcius() const{
    static_assert(s == 0 && m == 0 && kg == 0 && A == 0 && K == 1 && mol == 0 && cd == 0, "only statically avaiable on Temperature");
    return m_value + 273.15;
  };

  Metric &operator=(const Metric &o) {
    m_value = o.m_value;
    return *this;
  }
  Metric &operator=(const volatile Metric &o) volatile {
    m_value = o.m_value;
    return *this;
  }
  Metric &operator=(const volatile Metric &o) {
    m_value = o.m_value;
    return *this;
  }
  volatile Metric &operator=(const Metric &o) volatile {
    m_value = o.m_value;
    return *this;
  }

  Metric &operator=(Metric &&o) {
    m_value = o.m_value;
    return *this;
  }
  volatile Metric &operator=(volatile Metric &&o) volatile {
    m_value = o.m_value;
    return *this;
  }
  volatile Metric &operator=(Metric &&o) volatile {
    m_value = o.m_value;
    return *this;
  }
  Metric &operator=(volatile Metric &&o) {
    m_value = o.m_value;
    return *this;
  }

  Metric abs() const { return Metric(fabs(m_value)); }
  Metric abs() const volatile { return Metric(fabs(m_value)); }

  Metric &operator+=(const Metric &o) {
    m_value += o.m_value;
    return *this;
  }
  volatile Metric &operator+=(const volatile Metric &o) volatile {
    m_value += o.m_value;
    return *this;
  }
  volatile Metric &operator+=(const Metric &o) volatile {
    m_value += o.m_value;
    return *this;
  }
  Metric &operator+=(const volatile Metric &o) {
    m_value += o.m_value;
    return *this;
  }

  Metric &operator-=(const Metric &o) {
    m_value -= o.m_value;
    return *this;
  }
  volatile Metric &operator-=(const volatile Metric &o) volatile {
    m_value -= o.m_value;
    return *this;
  }
  volatile Metric &operator-=(const Metric &o) volatile {
    m_value -= o.m_value;
    return *this;
  }
  Metric &operator-=(const volatile Metric &o) {
    m_value -= o.m_value;
    return *this;
  }

  Metric &operator*=(const float &v) {
    m_value *= v;
    return *this;
  }
  volatile Metric &operator*=(const volatile float &v) volatile {
    m_value *= v;
    return *this;
  }
  volatile Metric &operator*=(const float &v) volatile {
    m_value *= v;
    return *this;
  }
  Metric &operator*=(const volatile float &v) {
    m_value *= v;
    return *this;
  }

  Metric &operator/=(const float &v) {
    m_value /= v;
    return *this;
  }
  volatile Metric &operator/=(const volatile float &v) volatile {
    m_value /= v;
    return *this;
  }
  volatile Metric &operator/=(const float &v) volatile {
    m_value /= v;
    return *this;
  }
  Metric &operator/=(const volatile float &v) {
    m_value /= v;
    return *this;
  }
  Metric &operator%=(const Metric &o) {
    m_value = fmod(m_value, o.m_value);
    return *this;
  }
  volatile Metric &operator%=(const volatile Metric &o) volatile {
    m_value = fmod(m_value, o.m_value);
    return *this;
  }
  Metric &operator%=(const volatile Metric &o) {
    m_value = fmod(m_value, o.m_value);
    return *this;
  }
  volatile Metric &operator%=(const Metric &o) volatile {
    m_value = fmod(m_value, o.m_value);
    return *this;
  }

  Metric rem(const Metric &o) { return Metric(remainder(m_value, o.m_value)); }
  Metric rem(const volatile Metric &o) volatile {
    return Metric(remainder(m_value, o.m_value));
  }
  Metric rem(const Metric &o) volatile {
    return Metric(remainder(m_value, o.m_value));
  }
  Metric rem(const volatile Metric &o) {
    return Metric(remainder(m_value, o.m_value));
  }

  explicit constexpr operator float() const { return m_value; }
  explicit constexpr operator float() const volatile { return m_value; }

  template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
  explicit constexpr
  operator Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>() const {
    return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(m_value);
  }
  template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
  explicit constexpr
  operator Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>() const volatile {
    return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(m_value);
  }

private:
  float m_value;
};

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) *
                                                       b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) *
                                                       b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) *
                                                       b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) *
                                                       b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const float &b,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return a * b;
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const volatile float &b,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return a * b;
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const float &b,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return a * b;
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator*(const volatile float &b,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return a * b;
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator*(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s + b_s, a_m + b_m, a_kg + b_kg, a_A + b_A, a_K + b_K,
                a_mol + b_mol, a_cd + b_cd>(static_cast<float>(a) *
                                            static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator*(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s + b_s, a_m + b_m, a_kg + b_kg, a_A + b_A, a_K + b_K,
                a_mol + b_mol, a_cd + b_cd>(static_cast<float>(a) *
                                            static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator*(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s + b_s, a_m + b_m, a_kg + b_kg, a_A + b_A, a_K + b_K,
                a_mol + b_mol, a_cd + b_cd>(static_cast<float>(a) *
                                            static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator*(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s + b_s, a_m + b_m, a_kg + b_kg, a_A + b_A, a_K + b_K,
                a_mol + b_mol, a_cd + b_cd>(static_cast<float>(a) *
                                            static_cast<float>(b));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) /
                                                       b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) /
                                                       b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) /
                                                       b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const float &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) /
                                                       b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const float &b,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<-a_s, -a_m, -a_kg, -a_A, -a_K, -a_mol, -a_cd>(
      b / static_cast<float>(a));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const volatile float &b,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<-a_s, -a_m, -a_kg, -a_A, -a_K, -a_mol, -a_cd>(
      b / static_cast<float>(a));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const float &b,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<-a_s, -a_m, -a_kg, -a_A, -a_K, -a_mol, -a_cd>(
      b / static_cast<float>(a));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const volatile float &b,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<-a_s, -a_m, -a_kg, -a_A, -a_K, -a_mol, -a_cd>(
      b / static_cast<float>(a));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) / static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) / static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) / static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator/(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) / static_cast<float>(b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator/(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s - b_s, a_m - b_m, a_kg - b_kg, a_A - b_A, a_K - b_K,
                a_mol - b_mol, a_cd - b_cd>(static_cast<float>(a) /
                                            static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator/(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s - b_s, a_m - b_m, a_kg - b_kg, a_A - b_A, a_K - b_K,
                a_mol - b_mol, a_cd - b_cd>(static_cast<float>(a) /
                                            static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator/(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s - b_s, a_m - b_m, a_kg - b_kg, a_A - b_A, a_K - b_K,
                a_mol - b_mol, a_cd - b_cd>(static_cast<float>(a) /
                                            static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd,
          int b_s, int b_m, int b_kg, int b_A, int b_K, int b_mol, int b_cd>
static constexpr auto
operator/(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<b_s, b_m, b_kg, b_A, b_K, b_mol, b_cd> &b) {
  return Metric<a_s - b_s, a_m - b_m, a_kg - b_kg, a_A - b_A, a_K - b_K,
                a_mol - b_mol, a_cd - b_cd>(static_cast<float>(a) /
                                            static_cast<float>(b));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator+(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) +
                                                       static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator+(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) +
                                                       static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator+(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) +
                                                       static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator+(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) +
                                                       static_cast<float>(b));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator-(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) -
                                                       static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator-(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) -
                                                       static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator-(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) -
                                                       static_cast<float>(b));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator-(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a) -
                                                       static_cast<float>(b));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator-(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(-static_cast<float>(a));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator-(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(-static_cast<float>(a));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator+(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return a;
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator+(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(static_cast<float>(a));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator%(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(
      fmod(static_cast<float>(a), static_cast<float>(b)));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator%(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(
      fmod(static_cast<float>(a), static_cast<float>(b)));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator%(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(
      fmod(static_cast<float>(a), static_cast<float>(b)));
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator%(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd>(
      fmod(static_cast<float>(a), static_cast<float>(b)));
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) < static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) < static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) < static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) < static_cast<float>(b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) > static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) > static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) > static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
          const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) > static_cast<float>(b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<=(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) <= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<=(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) <= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<=(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) <= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator<=(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) <= static_cast<float>(b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>=(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>=(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>=(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator>=(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator==(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator==(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator==(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator==(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}

template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator!=(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator!=(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator!=(const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}
template <int a_s, int a_m, int a_kg, int a_A, int a_K, int a_mol, int a_cd>
static constexpr auto
operator!=(const volatile Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &a,
           const Metric<a_s, a_m, a_kg, a_A, a_K, a_mol, a_cd> &b) {
  return static_cast<float>(a) >= static_cast<float>(b);
}

typedef Metric<1, 0, 0, 0, 0, 0, 0> Time;
// Unit Time.
typedef Metric<0, 1, 0, 0, 0, 0, 0> Distance;
// Unit kg.
typedef Metric<0, 0, 1, 0, 0, 0, 0> Weight;
// Unit: Ampere
typedef Metric<0, 0, 0, 1, 0, 0, 0> Current;
// Unit : Kelvin
typedef Metric<0, 0, 0, 0, 1, 0, 0> Temperature;
typedef Metric<-1, 0, 0, 0, 0, 0, 0> Frequency;
typedef Metric<-2, 1, 1, 0, 0, 0, 0> Force;
typedef Metric<-2, -1, 1, 0, 0, 0, 0> Pressure;
typedef Metric<-2, 2, 1, 0, 0, 0, 0> Energy;
typedef Metric<-3, 2, 1, 0, 0, 0, 0> Power;
typedef Metric<1, 0, 0, 1, 0, 0, 0> Charge;
typedef Metric<-3, 2, 1, -1, 0, 0, 0> Voltage;
typedef Metric<4, -2, 1, 2, 0, 0, 0> Capacity;
typedef Metric<-3, 2, 1, -2, 0, 0, 0> Resistance;
typedef Metric<3, -2, -1, 2, 0, 0, 0> Conductance;
typedef Metric<-2, 2, 1, -2, 0, 0, 0> Inductance;
typedef Metric<-1, 1, 0, 0, 0, 0, 0> Velocity;
typedef Metric<-2, 1, 0, 0, 0, 0, 0> Acceleration;
typedef Metric<0, 3, 0, 0, 0, 0, 0> Volume;
typedef Metric<0, 2, 0, 0, 0, 0, 0> Area;
typedef Metric<-1, 3, 0, 0, 0, 0, 0> FlowRate;

// Volume literals
constexpr Volume operator""_m3(const long double v) {
  return Volume(v);
}
constexpr Volume operator""_m3(const unsigned long long v) {
  return Volume(static_cast<float>(v));
}

constexpr Volume operator""_l(const long double v) {
  return Volume(v / 1e3);
}
constexpr Volume operator""_l(const unsigned long long v) {
  return Volume(static_cast<float>(v) / 1e3);
}

constexpr Volume operator""_ml(const long double v) {
  return Volume(v / 1e6);
}
constexpr Volume operator""_ml(const unsigned long long v) {
  return Volume(static_cast<float>(v) / 1e6);
}

// Time literals
constexpr Time operator""_h(const long double v) { return Time(v * 60 * 60); }
constexpr Time operator""_h(const unsigned long long v) {
  return Time(static_cast<float>(v) * 60 * 60);
}
constexpr Time operator""_min(const long double v) { return Time(v * 60); }
constexpr Time operator""_min(const unsigned long long v) {
  return Time(static_cast<float>(v) * 60);
}
constexpr Time operator""_s(const long double v) { return Time(v); }
constexpr Time operator""_s(const unsigned long long v) {
  return Time(static_cast<float>(v));
}
constexpr Time operator""_ms(const long double v) { return Time(v / 1e3); }
constexpr Time operator""_ms(const unsigned long long v) {
  return Time(static_cast<float>(v) / 1e3);
}
constexpr Time operator""_us(const long double v) { return Time(v / 1e6); }
constexpr Time operator""_us(const unsigned long long v) {
  return Time(static_cast<float>(v) / 1e9);
}
constexpr Time operator""_ns(const long double v) { return Time(v / 1e6); }
constexpr Time operator""_ns(const unsigned long long v) {
  return Time(static_cast<float>(v) / 1e9);
}


// Distance literals
constexpr Distance operator""_km(const long double v) {
  return Distance(v * 1e3);
}
constexpr Distance operator""_km(const unsigned long long v) {
  return Distance(static_cast<float>(v * 1e3));
}
constexpr Distance operator""_m(const long double v) { return Distance(v); }
constexpr Distance operator""_m(const unsigned long long v) {
  return Distance(static_cast<float>(v));
}
constexpr Distance operator""_cm(const long double v) {
  return Distance(v / 1e2);
}
constexpr Distance operator""_cm(const unsigned long long v) {
  return Distance(static_cast<float>(v) / 1e2);
}
constexpr Distance operator""_mm(const long double v) {
  return Distance(v / 1e3);
}
constexpr Distance operator""_mm(const unsigned long long v) {
  return Distance(static_cast<float>(v) / 1e3);
}
constexpr Distance operator""_um(const long double v) {
  return Distance(v / 1e6);
}
constexpr Distance operator""_um(const unsigned long long v) {
  return Distance(static_cast<float>(v) / 1e6);
}

// Weight literals
constexpr Weight operator""_kg(const long double v) { return Weight(v); }
constexpr Weight operator""_kg(const unsigned long long v) {
  return Weight(static_cast<float>(v));
}
constexpr Weight operator""_g(const long double v) { return Weight(v / 1e3); }
constexpr Weight operator""_g(const unsigned long long v) {
  return Weight(static_cast<float>(v / 1e3));
}
constexpr Weight operator""_mg(const long double v) { return Weight(v / 1e6); }
constexpr Weight operator""_mg(const unsigned long long v) {
  return Weight(static_cast<float>(v / 1e6));
}

// Current literals
constexpr Current operator""_kA(const long double v) {
  return Current(v * 1e3);
}
constexpr Current operator""_kA(const unsigned long long v) {
  return Current(static_cast<float>(v) * 1e3);
}
constexpr Current operator""_A(const long double v) { return Current(v); }
constexpr Current operator""_A(const unsigned long long v) {
  return Current(static_cast<float>(v));
}
constexpr Current operator""_mA(const long double v) {
  return Current(v / 1e3);
}
constexpr Current operator""_mA(const unsigned long long v) {
  return Current(static_cast<float>(v) / 1e3);
}
constexpr Current operator""_uA(const long double v) {
  return Current(v / 1e6);
}
constexpr Current operator""_uA(const unsigned long long v) {
  return Current(static_cast<float>(v) / 1e6);
}

// Temperature literals
constexpr Temperature operator""_K(const long double v) {
  return Temperature(v);
}
constexpr Temperature operator""_K(const unsigned long long v) {
  return Temperature(static_cast<float>(v));
}
constexpr Temperature operator""_Celcius(const long double v) {
  return Temperature(v + 273.15);
}
constexpr Temperature operator""_Celcius(const unsigned long long v) {
  return Temperature(static_cast<float>(v) + 273.15);
}

// Frequency literals
constexpr Frequency operator""_MHz(const long double v) {
  return Frequency(v * 1e6);
}
constexpr Frequency operator""_MHz(const unsigned long long v) {
  return Frequency(static_cast<float>(v) * 1e6);
}
constexpr Frequency operator""_kHz(const long double v) {
  return Frequency(v * 1e3);
}
constexpr Frequency operator""_kHz(const unsigned long long v) {
  return Frequency(static_cast<float>(v) * 1e3);
}
constexpr Frequency operator""_Hz(const long double v) { return Frequency(v); }
constexpr Frequency operator""_Hz(const unsigned long long v) {
  return Frequency(static_cast<float>(v));
}
constexpr Frequency operator""_mHz(const long double v) {
  return Frequency(v / 1e3);
}
constexpr Frequency operator""_mHz(const unsigned long long v) {
  return Frequency(static_cast<float>(v) / 1e3);
}

// Force literals.
constexpr Force operator""_kN(const long double v) { return Force(v * 1e3); }
constexpr Force operator""_kN(const unsigned long long v) {
  return Force(static_cast<float>(v) * 1e3);
}
constexpr Force operator""_N(const long double v) { return Force(v); }
constexpr Force operator""_N(const unsigned long long v) {
  return Force(static_cast<float>(v));
}
constexpr Force operator""_mN(const long double v) { return Force(v / 1e3); }
constexpr Force operator""_mN(const unsigned long long v) {
  return Force(static_cast<float>(v) / 1e3);
}
constexpr Force operator""_uN(const long double v) { return Force(v / 1e6); }
constexpr Force operator""_uN(const unsigned long long v) {
  return Force(static_cast<float>(v) / 1e6);
}

constexpr Pressure operator""_bar(const long double v) {
  return Pressure(v);
}
constexpr Pressure operator""_bar(const unsigned long long v) {
  return Pressure(static_cast<float>(v));
}

// Energy literals
constexpr Energy operator""_kJ(const long double v) { return Energy(v * 1e3); }
constexpr Energy operator""_kJ(const unsigned long long v) {
  return Energy(static_cast<float>(v) * 1e3);
}
constexpr Energy operator""_J(const long double v) { return Energy(v); }
constexpr Energy operator""_J(const unsigned long long v) {
  return Energy(static_cast<float>(v));
}
constexpr Energy operator""_mJ(const long double v) { return Energy(v / 1e3); }
constexpr Energy operator""_mJ(const unsigned long long v) {
  return Energy(static_cast<float>(v) / 1e3);
}
constexpr Energy operator""_uJ(const long double v) { return Energy(v / 1e6); }
constexpr Energy operator""_uJ(const unsigned long long v) {
  return Energy(static_cast<float>(v) / 1e6);
}

// Power literals
constexpr Power operator""_kW(const long double v) { return Power(v * 1e3); }
constexpr Power operator""_kW(const unsigned long long v) {
  return Power(static_cast<float>(v) * 1e3);
}
constexpr Power operator""_W(const long double v) { return Power(v); }
constexpr Power operator""_W(const unsigned long long v) {
  return Power(static_cast<float>(v));
}
constexpr Power operator""_mW(const long double v) { return Power(v / 1e3); }
constexpr Power operator""_mW(const unsigned long long v) {
  return Power(static_cast<float>(v) / 1e3);
}
constexpr Power operator""_uW(const long double v) { return Power(v / 1e6); }
constexpr Power operator""_uW(const unsigned long long v) {
  return Power(static_cast<float>(v) / 1e6);
}

// Charge literals.
constexpr Charge operator""_kC(const long double v) { return Charge(v * 1e3); }
constexpr Charge operator""_kC(const unsigned long long v) {
  return Charge(static_cast<float>(v) * 1e3);
}
constexpr Charge operator""_C(const long double v) { return Charge(v); }
constexpr Charge operator""_C(const unsigned long long v) {
  return Charge(static_cast<float>(v));
}
constexpr Charge operator""_mC(const long double v) { return Charge(v / 1e3); }
constexpr Charge operator""_mC(const unsigned long long v) {
  return Charge(static_cast<float>(v) / 1e3);
}
constexpr Charge operator""_uC(const long double v) { return Charge(v / 1e6); }
constexpr Charge operator""_uC(const unsigned long long v) {
  return Charge(static_cast<float>(v) / 1e6);
}

// Voltage literals
constexpr Voltage operator""_kV(const long double v) {
  return Voltage(v * 1e3);
}
constexpr Voltage operator""_kV(const unsigned long long v) {
  return Voltage(static_cast<float>(v) * 1e3);
}
constexpr Voltage operator""_V(const long double v) { return Voltage(v); }
constexpr Voltage operator""_V(const unsigned long long v) {
  return Voltage(static_cast<float>(v));
}
constexpr Voltage operator""_mV(const long double v) {
  return Voltage(v / 1e3);
}
constexpr Voltage operator""_mV(const unsigned long long v) {
  return Voltage(static_cast<float>(v) / 1e3);
}
constexpr Voltage operator""_uV(const long double v) {
  return Voltage(v / 1e6);
}
constexpr Voltage operator""_uV(const unsigned long long v) {
  return Voltage(static_cast<float>(v) / 1e6);
}

// Capacity literals
constexpr Capacity operator""_F(const long double v) { return Capacity(v); }
constexpr Capacity operator""_F(const unsigned long long v) {
  return Capacity(static_cast<float>(v));
}
constexpr Capacity operator""_mF(const long double v) {
  return Capacity(v / 1e3);
}
constexpr Capacity operator""_mF(const unsigned long long v) {
  return Capacity(static_cast<float>(v) / 1e3);
}
constexpr Capacity operator""_uF(const long double v) {
  return Capacity(v / 1e6);
}
constexpr Capacity operator""_uF(const unsigned long long v) {
  return Capacity(static_cast<float>(v) / 1e6);
}
constexpr Capacity operator""_nF(const long double v) {
  return Capacity(v / 1e9);
}
constexpr Capacity operator""_nF(const unsigned long long v) {
  return Capacity(static_cast<float>(v) / 1e9);
}

// Resistance literals
constexpr Resistance operator""_kOhm(const long double v) {
  return Resistance(v * 1e3);
}
constexpr Resistance operator""_kOhm(const unsigned long long v) {
  return Resistance(static_cast<float>(v) * 1e3);
}
constexpr Resistance operator""_Ohm(const long double v) {
  return Resistance(v);
}
constexpr Resistance operator""_Ohm(const unsigned long long v) {
  return Resistance(static_cast<float>(v));
}
constexpr Resistance operator""_mOhm(const long double v) {
  return Resistance(v / 1e3);
}
constexpr Resistance operator""_mOhm(const unsigned long long v) {
  return Resistance(static_cast<float>(v) / 1e3);
}
constexpr Resistance operator""_uOhm(const long double v) {
  return Resistance(v / 1e6);
}
constexpr Resistance operator""_uOhm(const unsigned long long v) {
  return Resistance(static_cast<float>(v) / 1e6);
}

// Conductance literals
constexpr Conductance operator""_MS(const long double v) {
  return Conductance(v * 1e6);
}
constexpr Conductance operator""_MS(const unsigned long long v) {
  return Conductance(static_cast<float>(v) * 1e6);
}
constexpr Conductance operator""_kS(const long double v) {
  return Conductance(v * 1e3);
}
constexpr Conductance operator""_kS(const unsigned long long v) {
  return Conductance(static_cast<float>(v) * 1e3);
}
constexpr Conductance operator""_S(const long double v) {
  return Conductance(v);
}
constexpr Conductance operator""_S(unsigned long long v) {
  return Conductance(static_cast<float>(v));
}
constexpr Conductance operator""_mS(const long double v) {
  return Conductance(v / 1e3);
}
constexpr Conductance operator""_mS(const unsigned long long v) {
  return Conductance(static_cast<float>(v / 1e3));
}
constexpr Conductance operator""_uS(const long double v) {
  return Conductance(v / 1e6);
}
constexpr Conductance operator""_uS(const unsigned long long v) {
  return Conductance(static_cast<float>(v / 1e6));
}

// Inductance literals
constexpr Inductance operator""_kH(const long double v) {
  return Inductance(v * 1e3);
}
constexpr Inductance operator""_kH(const unsigned long long v) {
  return Inductance(static_cast<float>(v) * 1e3);
}
constexpr Inductance operator""_H(const long double v) { return Inductance(v); }
constexpr Inductance operator""_H(const unsigned long long v) {
  return Inductance(static_cast<float>(v));
}
constexpr Inductance operator""_mH(const long double v) {
  return Inductance(v / 1e3);
}
constexpr Inductance operator""_mH(const unsigned long long v) {
  return Inductance(static_cast<float>(v) / 1e3);
}
constexpr Inductance operator""_uH(const long double v) {
  return Inductance(v / 1e6);
}
constexpr Inductance operator""_uH(const unsigned long long v) {
  return Inductance(static_cast<float>(v) / 1e6);
}

// Velocity literals
constexpr Velocity operator""_mps(const long double v) { return Velocity(v); }
constexpr Velocity operator""_mps(const unsigned long long v) {
  return Velocity(static_cast<float>(v));
}
constexpr Velocity operator""_kmph(const long double v) {
  return Velocity(v / 3.6);
}
constexpr Velocity operator""_kmph(const unsigned long long v) {
  return Velocity(static_cast<float>(v) / 3.6);
}

// Acceleration literals
constexpr Acceleration operator""_mps2(const long double v) {
  return Acceleration(v);
}
constexpr Acceleration operator""_mps2(const unsigned long long v) {
  return Acceleration(static_cast<float>(v));
}
