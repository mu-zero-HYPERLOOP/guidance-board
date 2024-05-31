#include <array>
#include <cassert>
#include <cmath>
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
      : m_elements({{v, static_cast<float>(std::forward<E>(elements))...}}) {}

  constexpr vec() : m_elements() {}

  constexpr vec(std::array<T, N> &&array) : m_elements(array) {}

  constexpr inline auto dot(const vec<T, N> &x) const -> float {
    float sum = 0;
    for (size_t i = 0; i < N; ++i) {
      sum += x.m_elements[i] * m_elements[i];
    }
    return sum;
  }

  __attribute__((always_inline)) constexpr auto operator[](size_t i) -> T & {
    assert(i < N);
    return m_elements[i];
  }

  __attribute__((always_inline)) constexpr auto operator[](size_t i) const
      -> const T & {
    assert(i < N);
    return m_elements[i];
  }

  inline constexpr auto operator+=(const vec<T, N> &x) -> vec<T, N> & {
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

  inline constexpr auto operator*=(const vec<T, N> &x) -> vec<T, N> & {
    for (size_t i = 0; i < N; ++i) {
      m_elements[i] *= x.m_elements[i];
    }
    return *this;
  }

  inline constexpr auto operator*=(const float &x) -> vec<T, N> & {
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

  inline constexpr auto operator/=(const float &x) -> vec<T, N> & {
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
constexpr inline auto operator-(const vec<T, N> &a, const vec<T, N> &b)
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
constexpr inline auto operator*(const float &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out *= a;
}

template <typename T, size_t N>
constexpr inline auto operator*(const vec<T, N> &b, const float &a)
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
constexpr inline auto operator/(const float &a, const vec<T, N> &b)
    -> vec<T, N> {
  vec<T, N> out = b;
  return out /= a;
}

template <typename T, size_t N>
constexpr inline auto operator/(const vec<T, N> &b, const float &a)
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
      vec<T, M> v, E &&...elements) noexcept
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

  inline constexpr auto operator*=(const float &x) -> mat<T, M, N> & {
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
inline constexpr auto operator*(const mat<T, M, N> &A, const float &x)
    -> mat<T, M, N> {
  mat<T, M, N> out = A;
  return out *= x;
}

template <typename T, size_t M, size_t N>
inline constexpr auto operator*(const float &x, const mat<T, M, N> &A)
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

  constexpr complex() : m_a(0.0f), m_b(0.0f) {}

  static constexpr auto from_polar(float r, float phi) {
    return complex(r * cos(phi), r * sin(phi));
  }

  __attribute__((always_inline)) constexpr auto re() -> float & { return m_a; }

  __attribute__((always_inline)) constexpr auto re() const -> const float & {
    return m_a;
  }

  __attribute__((always_inline)) constexpr auto im() -> float & { return m_b; }

  __attribute__((always_inline)) constexpr auto im() const -> const float & {
    return m_b;
  }

  __attribute__((always_inline)) constexpr auto conj() const -> complex {
    return complex(m_a, -m_b);
  };

  constexpr auto r() const -> float { return sqrtf(m_a * m_a + m_b * m_b); }

  constexpr auto phi() const -> float { return atan2(m_b, m_a); }

  constexpr auto pow(float p) const -> complex {
    float r = this->r();
    float phi = this->phi();
    return complex::from_polar(std::pow(r, p), phi * p);
  }

  constexpr auto root(float nth) const -> complex {
    float r = this->r();
    float phi = this->phi();
    return complex::from_polar(std::pow(r, 1.0f / nth), phi / nth);
  }

  constexpr auto operator*=(const complex &o) -> complex & {
    const float a = m_a * o.m_a - m_b * o.m_b;
    const float b = m_a * o.m_b + m_b * o.m_a;
    m_a = a;
    m_b = b;
    return *this;
  }

  constexpr auto operator/=(const complex &o) -> complex & {
    const float a = m_a * o.m_a + m_b * o.m_b;
    const float b = m_b * o.m_a - m_a * o.m_b;
    const float den = o.m_a * o.m_a + o.m_b * o.m_b;
    m_a = a / den;
    m_b = b / den;
    return *this;
  }

  constexpr auto operator+=(const complex &o) -> complex & {
    m_a += o.m_a;
    m_b += o.m_b;
    return *this;
  }

  constexpr auto operator-=(const complex &o) -> complex & {
    m_a -= o.m_a;
    m_b -= o.m_b;
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

constexpr auto operator/(const complex &x, const complex &y) -> complex {
  const float a = x.re() * y.re() + x.im() * y.im();
  const float b = x.im() * y.re() - x.re() * y.im();
  const float den = y.re() * y.re() + y.im() * y.im();
  return complex(a / den, b / den);
}

constexpr auto operator+(const complex &x, const complex &y) -> complex {
  return complex(x.re() + y.re(), x.im() + y.im());
}

constexpr auto operator-(const complex &x, const complex &y) -> complex {
  return complex(x.re() - y.re(), x.im() - y.im());
}

typedef float mediump;
typedef double highp;
typedef vec<mediump, 2> vec2;
typedef vec<mediump, 3> vec3;
typedef vec<mediump, 4> vec4;

typedef mat<mediump, 2, 2> mat2;
typedef mat<mediump, 3, 3> mat3;
typedef mat<mediump, 4, 4> mat4;

typedef mat<mediump, 2, 3> mat2x3;
typedef mat<mediump, 3, 2> mat3x2;
typedef mat<mediump, 2, 4> mat2x4;
typedef mat<mediump, 4, 2> mat4x2;
typedef mat<mediump, 3, 4> mat3x4;
typedef mat<mediump, 4, 3> mat4x3;
