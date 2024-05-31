#pragma once

template <typename T> class RectangleIntegral {
public:
  explicit RectangleIntegral(T inital) : m_integral(inital) {}

  void reset(T value) {
    m_integral = value;
  }

  inline void integrate(T v, float dt){
    m_integral += v * dt;
  }

  inline T get(){
    return m_integral;
  }

private:
  T m_integral;
};
