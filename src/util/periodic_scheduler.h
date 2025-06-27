#pragma once

#include "util/heap.h"
#include "util/metrics.h"
#include "util/pool_allocator.h"
#include <chrono>
#include <cstddef>

template <typename Job, size_t MAX_JOBS,
          typename Clock = std::chrono::high_resolution_clock>
struct PeriodicScheduler {
  using duration = typename Clock::duration;
  using time_point = typename Clock::time_point;
public:
  explicit PeriodicScheduler() : m_allocator(), m_min_heap() {}

  virtual ~PeriodicScheduler() = default;

  virtual void execute_job(const Job &job) = 0;

  bool schedule(const Time &period, const Job &job) {
    periodic_job *job_ptr = m_allocator.template alloc<periodic_job>();
    if (job_ptr == nullptr) {
      return false;
    }
    job_ptr->m_job = job;
    job_ptr->m_period = std::chrono::duration_cast<duration>(
        std::chrono::duration<float>(static_cast<float>(period)));
    return m_min_heap.insert(Clock::now(), job_ptr);
  }

  void update_continue() {
    const auto &next_job = m_min_heap.min();
    if (next_job.has_value()) {
      auto& [climax_ptr, periodic_job_ptr_ptr] = next_job.value();
      if (Clock::now() > *climax_ptr) {
        execute_job((*periodic_job_ptr_ptr)->m_job);
        m_min_heap.increase_key(0, *climax_ptr + (*periodic_job_ptr_ptr)->m_period);
      }
    }
  }

private:
  struct periodic_job {
    duration m_period;
    Job m_job;
  };
  PoolAllocator<sizeof(periodic_job), MAX_JOBS> m_allocator;
  MinHeap<time_point, periodic_job*, MAX_JOBS> m_min_heap;
};
