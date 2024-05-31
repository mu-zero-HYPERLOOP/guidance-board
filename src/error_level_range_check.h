#pragma once
#include "canzero/canzero.h"
#include "util/timestamp.h"

enum ErrorLevelRangeCheckOrd { EXPECT_UNDER, EXPECT_OVER };

template <ErrorLevelRangeCheckOrd ORD> class ErrorLevelRangeCheck {
public:
  ErrorLevelRangeCheck(float (*getter)(), error_level_config (*config_getter)(),
                       void (*error_setter)(error_level))
      : m_getter(getter), m_config_getter(config_getter),
        m_error_setter(error_setter), m_last_non_error(Timestamp::now()),
        m_last_non_warning(Timestamp::now()),
        m_last_non_info(Timestamp::now()) {
    m_error_setter(error_level_OK);
  }

  void check() {
    const float v = m_getter();
    const error_level_config &config = m_config_getter();
    const auto now = Timestamp::now();
    if constexpr (ORD == EXPECT_UNDER) {
      if (v < config.m_error_thresh) {
        m_last_non_error = now;
      }
      if (v < config.m_warning_thresh) {
        m_last_non_warning = now;
      }
      if (v < config.m_info_thresh) {
        m_last_non_info = now;
      }
    } else { // expect OVER
      if (v > config.m_error_thresh) {
        m_last_non_error = now;
      }
      if (v > config.m_warning_thresh) {
        m_last_non_warning = now;
      }
      if (v > config.m_info_thresh) {
        m_last_non_info = now;
      }
    }

    if (now - m_last_non_error > Duration::from_s(config.m_error_timeout)) {
      m_error_setter(error_level_ERROR);
    } else if (now - m_last_non_warning >
               Duration::from_s(config.m_warning_timeout)) {
      m_error_setter(error_level_WARNING);
    } else if (now - m_last_non_info >
               Duration::from_s(config.m_info_timeout)) {
      m_error_setter(error_level_INFO);
    } else {
      m_error_setter(error_level_OK);
    }
  }

private:
  float (*m_getter)();
  error_level_config (*m_config_getter)();
  void (*m_error_setter)(error_level);
  Timestamp m_last_non_error;
  Timestamp m_last_non_warning;
  Timestamp m_last_non_info;
};
