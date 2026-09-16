// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__THREAD_CPU_COUNTER_HPP_
#define SLAM_3D_LOCALIZATION__THREAD_CPU_COUNTER_HPP_

#include <atomic>
#include <cstdint>
#include <ctime>

namespace gole::slam_3d_localization
{

// Cumulative calling-thread CPU time: excludes blocking and descheduled time.
// Nested scopes intentionally overlap; their totals must not be summed.
class ThreadCpuCounter
{
public:
  class Scope
  {
  public:
    explicit Scope(ThreadCpuCounter & counter) noexcept
    : counter_(counter), valid_start_(readNs(start_ns_)) {}

    ~Scope() noexcept
    {
      std::uint64_t end_ns = 0U;
      if (valid_start_ && readNs(end_ns) && end_ns >= start_ns_) {
        counter_.nanoseconds_.fetch_add(end_ns - start_ns_, std::memory_order_relaxed);
      } else {
        counter_.clock_failures_.fetch_add(1U, std::memory_order_relaxed);
      }
      counter_.completed_scopes_.fetch_add(1U, std::memory_order_relaxed);
    }

    Scope(const Scope &) = delete;
    Scope & operator=(const Scope &) = delete;
    Scope(Scope &&) = delete;
    Scope & operator=(Scope &&) = delete;

  private:
    static bool readNs(std::uint64_t & output) noexcept
    {
      timespec sample{};
      if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &sample) != 0 || sample.tv_sec < 0) {
        return false;
      }
      output = static_cast<std::uint64_t>(sample.tv_sec) * 1000000000ULL +
        static_cast<std::uint64_t>(sample.tv_nsec);
      return true;
    }

    ThreadCpuCounter & counter_;
    std::uint64_t start_ns_{0U};
    bool valid_start_;
  };

  double ms() const noexcept
  {
    return static_cast<double>(nanoseconds_.load(std::memory_order_relaxed)) * 1.0e-6;
  }
  std::uint64_t count() const noexcept
  {
    return completed_scopes_.load(std::memory_order_relaxed);
  }
  std::uint64_t clockFailures() const noexcept
  {
    return clock_failures_.load(std::memory_order_relaxed);
  }

private:
  std::atomic<std::uint64_t> nanoseconds_{0U};
  std::atomic<std::uint64_t> completed_scopes_{0U};
  std::atomic<std::uint64_t> clock_failures_{0U};
};

}  // namespace gole::slam_3d_localization
#endif  // SLAM_3D_LOCALIZATION__THREAD_CPU_COUNTER_HPP_
