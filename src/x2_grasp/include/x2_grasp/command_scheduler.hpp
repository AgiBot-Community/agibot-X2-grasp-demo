#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>

namespace x2_grasp {

struct ScheduleMetrics {
  std::size_t frames_requested{0};
  std::size_t frames_published{0};
  std::size_t deadline_misses{0};
  std::int64_t max_lateness_ns{0};
  double elapsed_seconds{0.0};
  bool canceled{false};
  bool watchdog_triggered{false};
  bool hold_published{false};
};

class PeriodicScheduler {
 public:
  using PublishCallback = std::function<void(std::size_t, double)>;

  PeriodicScheduler(double deadline_tolerance_seconds,
                    double watchdog_seconds);

  ScheduleMetrics run(std::size_t frame_count, double duration_seconds,
                      const PublishCallback &publish);
  void reset_cancel() noexcept;
  void cancel() noexcept;
  bool busy() const noexcept;

 private:
  std::chrono::steady_clock::duration deadline_tolerance_;
  std::chrono::steady_clock::duration watchdog_limit_;
  std::atomic_bool cancel_requested_{false};
  std::atomic_bool busy_{false};
  std::mutex run_mutex_;
};

std::size_t smoothstep_frame_count(double duration_seconds, double rate_hz,
                                   double max_abs_delta,
                                   double max_delta_per_step);
double smoothstep_progress(std::size_t index, std::size_t frame_count);

}  // namespace x2_grasp
