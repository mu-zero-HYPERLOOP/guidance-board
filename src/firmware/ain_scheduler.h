#pragma once

#include "firmware/guidance_board.h"
#include "firmware/pinout.h"
#include "util/periodic_scheduler.h"
#include <chrono>


struct AinSchedulerJob {
  ain_pin pin;
  void (*on_value)(const Voltage&);
};

template<size_t MAX_JOBS, typename Clock = std::chrono::high_resolution_clock>
struct AinScheduler : PeriodicScheduler<AinSchedulerJob, MAX_JOBS, Clock>{
  public:
    AinScheduler() = default;

    void execute_job(const AinSchedulerJob& job) override {
      Voltage v = guidance_board::sync_read(job.pin);
      job.on_value(v);
    }

    bool register_periodic(const Time& period, ain_pin pin, 
        void (*on_value)(const Voltage&)) {
      AinSchedulerJob job;
      job.pin = pin;
      job.on_value = on_value;
      return this->schedule(period, job);
    }
};
