#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <stddef.h>
#include <type_traits>
#include <utility>

#include <type_traits>

template <typename T, size_t N> struct vec {

  template <typename... E,
            typename = typename std::enable_if_t<
                std::conjunction_v<std::is_convertible<E, T>...> &&
                    N == (1 + sizeof...(E)),
                void>>
  __attribute__((always_inline)) constexpr explicit vec(
      T v, E &&...elements) noexcept
      : m_elements({{v, static_cast<T>(std::forward<E>(elements))...}}) {}

  constexpr vec() : m_elements() {}

  constexpr vec(std::array<T, N> &&array) : m_elements(array) {}

  constexpr inline auto dot(const vec<T, N> &x) const -> T {
    T sum = 0;
    for (size_t i = 0; i < N; ++i) {
      sum += x.m_elements[i] * m_elements[i];
    }
    return sum;
  }
  constexpr inline auto dot(const vec<T, N> &x) volatile const -> T {
    T sum = 0;
    for (size_t i = 0; i < N; ++i) {
      sum += x.m_elements[i] * m_elements[i];
    }
    return sum;
  }

  constexpr inline auto mag() const -> T {
    T sum = 0;
    for (size_t i = 0; i < N; ++i) {
      sum += m_elements[i] * m_elements[i];
    }
    return sum;
  }
  constexpr inline auto mag() volatile const -> T {
    T sum = 0;
    for (size_t i = 0; i < N; ++i) {
      sum += m_elements[i] * m_elements[i];
    }
    return sum;
  }

  constexpr inline auto cross(const vec<T,N> &o) const -> T {
    static_assert(N == 3);
    return vec<T, N>(y() * o.z() - z() * o.y(), z() * o.x() - x() * o.z(), x() * o.y() - y() * o.x());
  }
  constexpr inline auto cross(const vec<T,N> &o) volatile const -> T {
    static_assert(N == 3);
    return vec<T, N>(y() * o.z() - z() * o.y(), z() * o.x() - x() * o.z(), x() * o.y() - y() * o.x());
  }

  __attribute__((always_inline)) constexpr auto operator[](size_t i) -> T & {
    assert(i < N);
    return m_elements[i];
  }
  __attribute__((always_inline)) constexpr auto operator[](size_t i) volatile -> T & {
    assert(i < N);
    return m_elements[i];
  }

  __attribute__((always_inline)) constexpr auto operator[](size_t i) const
      -> const T & {
    assert(i < N);
    return m_elements[i];
  }
  __attribute__((always_inline)) constexpr auto operator[](size_t i) volatile const
      -> const T & {
    assert(i < N);
    return m_elements[i];
  }

  constexpr auto x() const -> const T & {
    static_assert(N >= 1);
    return m_elements[0];
  }
  constexpr auto x() volatile const -> volatile const T & {
    static_assert(N >= 1);
    return m_elements[0];
  }
  constexpr auto y() const -> const T & {
    static_assert(N >= 2);
    return m_elements[1];
  }
  constexpr auto y() volatile const -> volatile const T & {
    static_assert(N >= 2);
    return m_elements[1];
  }
  constexpr auto z() const -> const T & {
    static_assert(N >= 3);
    return m_elements[1];
  }
  constexpr auto z() volatile const -> volatile const T & {
    static_assert(N >= 3);
    return m_elements[1];
  }
  constexpr auto w() const -> const T & {
    static_assert(N >= 4);
    return m_elements[1];
  }
  constexpr auto w() volatile const -> volatile const T & {
    static_assert(N >= 4);
    return m_elements[1];
  }
  constexpr auto x() -> T & {
    static_assert(N >= 1);
    return m_elements[0];
  }
  constexpr auto x() volatile -> volatile T & {
    static_assert(N >= 1);
    return m_elements[0];
  }
  constexpr auto y() -> T & {
    static_assert(N >= 2);
    return m_elements[1];
  }
  constexpr auto y() volatile -> volatile T & {
    static_assert(N >= 2);
    return m_elements[1];
  }
  constexpr auto z() -> T & {
    static_assert(N >= 3);
    return m_elements[1];
  }
  constexpr auto z() volatile -> volatile T & {
    static_assert(N >= 3);
    return m_elements[1];
  }
  constexpr auto w() -> T & {
    static_assert(N >= 4);
    return m_elements[1];
  }
  constexpr auto w() volatile -> volatile T & {
    static_assert(N >= 4);
    return m_elements[1];
  }

  inline constexpr auto operator+=(const vec<T, N> &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] += x.m_elements[i];
    }
    return *this;
  }
  inline constexpr auto operator+=(const vec<T, N> &x) volatile -> volatile vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] += x.m_elements[i];
    }
    return *this;
  }

  inline constexpr auto operator-=(const vec<T, N> &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] -= x.m_elements[i];
    }
    return *this;
  }
  inline constexpr auto operator-=(const vec<T, N> &x) volatile -> volatile vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] -= x.m_elements[i];
    }
    return *this;
  }

  inline constexpr auto operator*=(const vec<T, N> &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] *= x.m_elements[i];
    }
    return *this;
  }
  inline constexpr auto operator*=(const vec<T, N> &x) volatile -> volatile vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] *= x.m_elements[i];
    }
    return *this;
  }

  inline constexpr auto operator*=(const T &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] *= x;
    }
    return *this;
  }
  inline constexpr auto operator*=(const T &x) volatile -> volatile vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] *= x;
    }
    return *this;
  }

  inline constexpr auto operator/=(const vec<T, N> &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] /= x.m_elements[i];
    }
    return *this;
  }
  inline constexpr auto operator/=(volatile const vec<T, N> &x) -> volatile vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] /= x.m_elements[i];
    }
    return *this;
  }

  inline constexpr auto operator/=(const T &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] /= x;
    }
    return *this;
  }
  inline constexpr auto operator/=(volatile const T &x) -> volatile vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] /= x;
    }
    return *this;
  }

private:
  std::array<T, N> m_elements;
};

template <typename T, size_t N>
constexpr inline auto operator+(const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out += b;
}
template <typename T, size_t N>
constexpr inline auto operator+(const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out += b;
}
template <typename T, size_t N>
constexpr inline auto operator+(volatile const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out += b;
}
template <typename T, size_t N>
constexpr inline auto operator+(volatile const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out += b;
}

template <typename T, size_t N>
constexpr inline auto operator-(const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out -= b;
}
template <typename T, size_t N>
constexpr inline auto operator-(const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out -= b;
}
template <typename T, size_t N>
constexpr inline auto operator-(volatile const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out -= b;
}
template <typename T, size_t N>
constexpr inline auto operator-(volatile const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out -= b;
}

template <typename T, size_t N>
constexpr inline auto operator*(const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out *= b;
}
template <typename T, size_t N>
constexpr inline auto operator*(const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out *= b;
}
template <typename T, size_t N>
constexpr inline auto operator*(volatile const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out *= b;
}
template <typename T, size_t N>
constexpr inline auto operator*(volatile const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out *= b;
}

template <typename T, size_t N>
constexpr inline auto operator*(const T &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}
template <typename T, size_t N>
constexpr inline auto operator*(const T &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}
template <typename T, size_t N>
constexpr inline auto operator*(volatile const T &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}
template <typename T, size_t N>
constexpr inline auto operator*(volatile const T &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}

template <typename T, size_t N>
constexpr inline auto operator*(const vec<T, N> &b, const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}
template <typename T, size_t N>
constexpr inline auto operator*(const vec<T, N> &b, volatile const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}
template <typename T, size_t N>
constexpr inline auto operator*(volatile const vec<T, N> &b, const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}
template <typename T, size_t N>
constexpr inline auto operator*(volatile const vec<T, N> &b, volatile const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}

template <typename T, size_t N>
constexpr inline auto operator/(const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out /= b;
}
template <typename T, size_t N>
constexpr inline auto operator/(const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out /= b;
}
template <typename T, size_t N>
constexpr inline auto operator/(volatile const vec<T, N> &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out /= b;
}
template <typename T, size_t N>
constexpr inline auto operator/(volatile const vec<T, N> &a, volatile const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = a;
  return out /= b;
}


template <typename T, size_t N>
constexpr inline auto operator/(const vec<T, N> &b, const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out /= a;
}
template <typename T, size_t N>
constexpr inline auto operator/(const vec<T, N> &b, volatile const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out /= a;
}
template <typename T, size_t N>
constexpr inline auto operator/(volatile const vec<T, N> &b, const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out /= a;
}
template <typename T, size_t N>
constexpr inline auto operator/(volatile const vec<T, N> &b, volatile const T &a)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out /= a;
}

template <typename T, size_t M, size_t N> struct mat {
  std::array<vec<T, M>, N> m_elements;

  template <typename... E,
            typename = typename std::enable_if_t<
                std::conjunction_v<std::is_same<E, vec<T, M>>...> &&
                    N == (1 + sizeof...(E)),
                void>>
  __attribute__((always_inline)) constexpr inline explicit mat(
      vec<T, M> v, E... elements) noexcept
      : m_elements({{v, std::forward<E>(elements)...}}) {}

  inline explicit mat(std::array<vec<T, M>, N> &&elements)
      : m_elements(elements) {}

  __attribute__((always_inline)) constexpr auto operator[](size_t i)
      -> vec<T, M> & {
    return m_elements[i];
  }

  __attribute__((always_inline)) constexpr auto operator[](size_t i) const
      -> const vec<T, M> & {
    return m_elements[i];
  }

  inline constexpr auto operator*=(const T &x) -> mat<T, M, N> & {
    for (size_t n = 0; n < N; ++n) {
      m_elements[n] *= x;
    }
    return *this;
  }

  inline constexpr auto operator*=(const mat<T, M, N> &x) -> mat<T, M, N> & {
    for (size_t n = 0; n < N; ++n) {
      m_elements[n] *= x.m_elements[n];
    }
    return *this;
  }

  inline constexpr auto operator/=(const mat<T, M, N> &x) -> mat<T, M, N> & {
    for (size_t n = 0; n < N; ++n) {
      m_elements[n] /= x.m_elements[n];
    }
    return *this;
  }

  inline constexpr auto operator+=(const mat<T, M, N> &x) -> mat<T, M, N> & {
    for (size_t n = 0; n < N; ++n) {
      m_elements[n] += x.m_elements[n];
    }
    return *this;
  }

  inline constexpr auto operator-=(const mat<T, M, N> &x) -> mat<T, M, N> & {
    for (size_t n = 0; n < N; ++n) {
      m_elements[n] -= x.m_elements[n];
    }
    return *this;
  }
};

template <typename T, size_t M, size_t N>
inline constexpr auto operator*(const mat<T, M, N> &A, const vec<T, N> &x)
    -> vec<T, M> {
  vec<T, M> out;
  for (size_t n = 0; n < N; ++n) {
    out += A[n] * x[n];
  }
  return out;
}

template <typename T, size_t M, size_t K, size_t N>
inline constexpr auto operator*(const mat<T, M, K> &A, const mat<T, K, N> &B)
    -> mat<T, M, N> {
  mat<T, M, N> out;
  for (size_t m = 0; m < M; ++m) {
    for (size_t n = 0; n < N; ++n) {
      for (size_t k = 0; k < K; ++k) {
        out[m][n] += A.m_elements[m][k] * B.m_elements[k][n];
      }
    }
  }
  return out;
}

template <typename T, size_t M, size_t N>
inline constexpr auto operator*(const mat<T, M, N> &A, const T &x)
    -> mat<T, M, N> {
  mat<T, M, N> out = A;
  return out *= x;
}

template <typename T, size_t M, size_t N>
inline constexpr auto operator*(const T &x, const mat<T, M, N> &A)
    -> mat<T, M, N> {
  mat<T, M, N> out = A;
  return out *= x;
}

template <typename T, size_t M, size_t N>
inline constexpr auto operator+(const mat<T, M, N> &A, const mat<T, M, N> &B)
    -> vec<T, M> {
  vec<T, M> out;
  for (size_t n = 0; n < N; ++n) {
    out[n] = A[n] + B[n];
  }
  return out;
}

template <typename T, size_t M, size_t N>
inline constexpr auto operator-(const mat<T, M, N> &A, const mat<T, M, N> &B)
    -> vec<T, M> {
  vec<T, M> out;
  for (size_t n = 0; n < N; ++n) {
    out[n] = A[n] - B[n];
  }
  return out;
}

struct complex {

  explicit constexpr complex(float a, float b) : m_a(a), m_b(b) {}
  constexpr complex(vec<float,2> v) : m_a(v.x()), m_b(v.y()) {}

  constexpr complex() : m_a(0.0f), m_b(0.0f) {}

  complex(const complex& o) : m_a(o.m_a), m_b(o.m_b){
  }
  complex(volatile const complex& o) : m_a(o.m_a), m_b(o.m_b){
  }
  complex(complex&& o) : m_a(o.m_a), m_b(o.m_b){
  }
  complex(volatile complex&& o) : m_a(o.m_a), m_b(o.m_b){
  }

  static constexpr auto from_polar(float r, float phi) {
    // should get compiled as a sincosf
    float sin = std::sin(phi), cos = std::cos(phi);
    return complex(r * sin, r * cos);
  }

  constexpr auto as_vec() const -> vec<float, 2> {
    return vec<float, 2>(m_a, m_b);
  }
  constexpr auto as_vec() volatile const -> vec<float, 2> {
    return vec<float, 2>(m_a, m_b);
  }

  constexpr operator vec<float, 2>() const { return vec<float, 2>(m_a, m_b); }
  constexpr operator vec<float, 2>() volatile const {
    return vec<float, 2>(m_a, m_b);
  }

  __attribute__((always_inline)) constexpr auto re() -> float & { return m_a; }

  __attribute__((always_inline)) constexpr auto re() const -> const float & {
    return m_a;
  }

  __attribute__((always_inline)) constexpr auto re() volatile
      -> volatile float & {
    return m_a;
  }

  __attribute__((always_inline)) constexpr auto re() volatile const
      -> float  {
    return m_a;
  }

  __attribute__((always_inline)) constexpr auto im() -> float & { return m_b; }

  __attribute__((always_inline)) constexpr auto im() const -> const float & {
    return m_b;
  }

  __attribute__((always_inline)) constexpr auto im() volatile
      -> volatile float & {
    return m_b;
  }

  __attribute__((always_inline)) constexpr auto im() volatile const
      -> float {
    return m_b;
  }

  __attribute__((always_inline)) constexpr auto conj() const -> complex {
    return complex(re(), -im());
  };

  __attribute__((always_inline)) constexpr auto conj() volatile const
      -> complex {
    return complex(re(), -im());
  };

  constexpr inline auto abs() const -> float {
    return std::sqrt(re() * re() + im() * im());
  }
  constexpr inline auto abs() volatile const -> float {
    return std::sqrt(re() * re() + im() * im());
  }

  constexpr inline auto norm() const -> float { return m_a * m_a + m_b * m_b; }
  constexpr inline auto norm() volatile const -> float {
    return re() * re() + im() * im();
  }

  constexpr inline auto arg() const -> float { return std::atan2(m_b, m_a); }
  constexpr inline auto arg() volatile const -> float {
    return std::atan2(im(), re());
  }

  constexpr inline auto operator*=(const complex &o) -> complex & {
    const float a = re() * o.re() - im() * o.im();
    const float b = re() * o.im() + im() * o.re();
    m_a = a;
    m_b = b;
    return *this;
  }

  constexpr inline auto operator*=(const complex &o) volatile
      -> volatile complex & {
    const float a = re() * o.re() - im() * o.im();
    const float b = re() * o.im() + im() * o.re();
    m_a = a;
    m_b = b;
    return *this;
  }

  constexpr inline auto operator*=(volatile const complex &o) volatile
      -> volatile complex & {
    const float a = re() * o.re() - im() * o.im();
    const float b = re() * o.im() + im() * o.re();
    m_a = a;
    m_b = b;
    return *this;
  }

  constexpr inline auto operator/=(const complex &o) -> complex & {
    const float a = m_a * o.m_a + m_b * o.m_b;
    const float b = m_b * o.m_a - m_a * o.m_b;
    const float den = o.m_a * o.m_a + o.m_b * o.m_b;
    m_a = a / den;
    m_b = b / den;
    return *this;
  }

  constexpr inline auto operator/=(const complex &o) volatile
      -> volatile complex & {
    const float a = re() * o.re() + im() * o.im();
    const float b = im() * o.re() - re() * o.im();
    const float den = o.re() * o.re() + o.im() * o.im();
    m_a = a / den;
    m_b = b / den;
    return *this;
  }
  constexpr inline auto operator/=(volatile const complex &o) volatile
      -> volatile complex & {
    const float a = re() * o.re() + im() * o.im();
    const float b = im() * o.re() - re() * o.im();
    const float den = o.re() * o.re() + o.im() * o.im();
    m_a = a / den;
    m_b = b / den;
    return *this;
  }

  constexpr auto operator+=(const complex &o) -> complex & {
    m_a += o.re();
    m_b += o.im();
    return *this;
  }
  constexpr auto operator+=(const complex &o) volatile -> volatile complex & {
    m_a += o.re();
    m_b += o.im();
    return *this;
  }
  constexpr auto operator+=(volatile const complex &o) volatile
      -> volatile complex & {
    m_a += o.re();
    m_b += o.im();
    return *this;
  }

  constexpr auto operator-=(const complex &o) -> complex & {
    m_a -= o.re();
    m_b -= o.im();
    return *this;
  }
  constexpr auto operator-=(const complex &o) volatile -> volatile complex & {
    m_a -= o.re();
    m_b -= o.im();
    return *this;
  }
  constexpr auto operator-=(volatile const complex &o) volatile
      -> volatile complex & {
    m_a = re() + o.re();
    m_b = im() + o.im();
    return *this;
  }

private:
  float m_a;
  float m_b;
};

constexpr auto operator*(const complex &x, const complex &y) -> complex {
  return complex(x.re() * y.re() - x.im() * y.im(),
                 x.re() * y.im() + x.im() * y.re());
}
constexpr auto operator*(const complex &x, volatile const complex &y)
    -> complex {
  return complex(x.re() * y.re() - x.im() * y.im(),
                 x.re() * y.im() + x.im() * y.re());
}
constexpr auto operator*(volatile const complex &x, const complex &y)
    -> complex {
  return complex(x.re() * y.re() - x.im() * y.im(),
                 x.re() * y.im() + x.im() * y.re());
}
constexpr auto operator*(volatile const complex &x, volatile const complex &y)
    -> complex {
  return complex(x.re() * y.re() - x.im() * y.im(),
                 x.re() * y.im() + x.im() * y.re());
}

constexpr auto operator/(const complex &x, const complex &y) -> complex {
  const float a = x.re() * y.re() + x.im() * y.im();
  const float b = x.im() * y.re() - x.re() * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}
constexpr auto operator/(const complex &x, volatile const complex &y)
    -> complex {
  const float a = x.re() * y.re() + x.im() * y.im();
  const float b = x.im() * y.re() - x.re() * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}
constexpr auto operator/(volatile const complex &x, const complex &y)
    -> complex {
  const float a = x.re() * y.re() + x.im() * y.im();
  const float b = x.im() * y.re() - x.re() * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}
constexpr auto operator/(volatile const complex &x, volatile const complex &y)
    -> complex {
  const float a = x.re() * y.re() + x.im() * y.im();
  const float b = x.im() * y.re() - x.re() * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}

constexpr auto operator+(const complex &x, const complex &y) -> complex {
  return complex(x.re() + y.re(), x.im() + y.im());
}
constexpr auto operator+(const complex &x, volatile const complex &y)
    -> complex {
  return complex(x.re() + y.re(), x.im() + y.im());
}
constexpr auto operator+(volatile const complex &x, const complex &y)
    -> complex {
  return complex(x.re() + y.re(), x.im() + y.im());
}
constexpr auto operator+(volatile const complex &x, volatile const complex &y)
    -> complex {
  return complex(x.re() + y.re(), x.im() + y.im());
}

constexpr auto operator-(const complex &x, const complex &y) -> complex {
  return complex(x.re() - y.re(), x.im() - y.im());
}
constexpr auto operator-(const complex &x, volatile const complex &y)
    -> complex {
  return complex(x.re() - y.re(), x.im() - y.im());
}
constexpr auto operator-(volatile const complex &x, const complex &y)
    -> complex {
  return complex(x.re() - y.re(), x.im() - y.im());
}
constexpr auto operator-(volatile const complex &x, volatile const complex &y)
    -> complex {
  return complex(x.re() - y.re(), x.im() - y.im());
}

constexpr auto operator*(const complex &x, const float &y) -> complex {
  return complex(x.re() * y, x.im() * y);
}
constexpr auto operator*(const complex &x, float y) -> complex {
  return complex(x.re() * y, x.im() * y);
}
constexpr auto operator*(volatile const complex &x, const float &y) -> complex {
  return complex(x.re() * y, x.im() * y);
}
constexpr auto operator*(volatile const complex &x, float y)
    -> complex {
  return complex(x.re() * y, x.im() * y);
}

constexpr auto operator*(const float &x, const complex &y) -> complex {
  return complex(y.re() * x, y.im() * x);
}
constexpr auto operator*(const float &x, volatile const complex &y) -> complex {
  return complex(y.re() * x, y.im() * x);
}
constexpr auto operator*(float x, const complex &y) -> complex {
  return complex(y.re() * x, y.im() * x);
}
constexpr auto operator*(float x, volatile const complex &y)
    -> complex {
  return complex(y.re() * x, y.im() * x);
}

constexpr auto operator/(const complex &x, const float &y) -> complex {
  return complex(x.re() / y, x.im() / y);
}
constexpr auto operator/(const complex &x, float y) -> complex {
  return complex(x.re() / y, x.im() / y);
}
constexpr auto operator/(volatile const complex &x, const float &y) -> complex {
  return complex(x.re() / y, x.im() / y);
}
constexpr auto operator/(volatile const complex &x, float y)
    -> complex {
  return complex(x.re() / y, x.im() / y);
}

constexpr auto operator/(const float &x, const complex &y) -> complex {
  const float a = x * y.re();
  const float b = x * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}
constexpr auto operator/(const float &x, volatile const complex &y) -> complex {
  const float a = x * y.re();
  const float b = x * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}
constexpr auto operator/(float x, const complex &y) -> complex {
  const float a = x * y.re();
  const float b = x * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}
constexpr auto operator/(float x, volatile const complex &y)
    -> complex {
  const float a = x * y.re();
  const float b = x * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}

constexpr auto operator+(const complex &x, const float &y) -> complex {
  return complex(x.re() + y, x.im());
}
constexpr auto operator+(const complex &x, float y) -> complex {
  return complex(x.re() + y, x.im());
}
constexpr auto operator+(volatile const complex &x, const float &y) -> complex {
  return complex(x.re() + y, x.im());
}
constexpr auto operator+(volatile const complex &x, float y)
    -> complex {
  return complex(x.re() + y, x.im());
}

constexpr auto operator+(const float &x, const complex &y) -> complex {
  return complex(y.re() + x, y.im());
}
constexpr auto operator+(const float &x, volatile const complex &y) -> complex {
  return complex(y.re() + x, y.im());
}
constexpr auto operator+(float x, const complex &y) -> complex {
  return complex(y.re() + x, y.im());
}
constexpr auto operator+(float x, volatile const complex &y)
    -> complex {
  return complex(y.re() + x, y.im());
}

constexpr auto operator-(const complex &x, const float &y) -> complex {
  return complex(x.re() - y, x.im());
}
constexpr auto operator-(const complex &x, float y) -> complex {
  return complex(x.re() - y, x.im());
}
constexpr auto operator-(volatile const complex &x, const float &y) -> complex {
  return complex(x.re() - y, x.im());
}
constexpr auto operator-(volatile const complex &x, float y)
    -> complex {
  return complex(x.re() - y, x.im());
}

constexpr auto operator-(const float &x, const complex &y) -> complex {
  return complex(x - y.re(), y.im());
}
constexpr auto operator-(const float &x, volatile const complex &y) -> complex {
  return complex(x - y.re(), y.im());
}
constexpr auto operator-(float x, const complex &y) -> complex {
  return complex(x - y.re(), y.im());
}
constexpr auto operator-(float x, volatile const complex &y)
    -> complex {
  return complex(x - y.re(), y.im());
}

constexpr auto operator-(const complex &x) -> complex {
  return complex(-x.re(), -x.im());
}
constexpr auto operator-(volatile const complex &x) -> complex {
  return complex(-x.re(), -x.im());
}

constexpr auto operator+(const complex &x) -> complex {
  return complex(x.re(), x.im());
}
constexpr auto operator+(volatile const complex &x) -> complex {
  return complex(x.re(),x.im());
}

typedef float mediump;
typedef double highp;
typedef vec<mediump, 2> vec2;
typedef vec<mediump, 3> vec3;
typedef vec<mediump, 4> vec4;
typedef vec<highp, 2> pvec2;
typedef vec<highp, 3> pvec3;
typedef vec<highp, 4> pvec4;

typedef mat<mediump, 2, 2> mat2;
typedef mat<mediump, 3, 3> mat3;
typedef mat<mediump, 4, 4> mat4;

typedef mat<highp, 2, 2> pmat2;
typedef mat<highp, 3, 3> pmat3;
typedef mat<highp, 4, 4> pmat4;

typedef mat<mediump, 2, 3> mat2x3;
typedef mat<mediump, 3, 2> mat3x2;
typedef mat<mediump, 2, 4> mat2x4;
typedef mat<mediump, 4, 2> mat4x2;
typedef mat<mediump, 3, 4> mat3x4;
typedef mat<mediump, 4, 3> mat4x3;

typedef mat<highp, 2, 3> pmat2x3;
typedef mat<highp, 3, 2> pmat3x2;
typedef mat<highp, 2, 4> pmat2x4;
typedef mat<highp, 4, 2> pmat4x2;
typedef mat<highp, 3, 4> pmat3x4;
typedef mat<highp, 4, 3> pmat4x3;
