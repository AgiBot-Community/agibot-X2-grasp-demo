#include "x2_grasp/command_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

namespace x2_grasp {
namespace {

class BusyGuard {
 public:
  explicit BusyGuard(std::atomic_bool &busy) : busy_(busy) {
    busy_.store(true, std::memory_order_release);
  }
  ~BusyGuard() { busy_.store(false, std::memory_order_release); }

 private:
  std::atomic_bool &busy_;
};

void require_positive_finite(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be a positive finite value");
  }
}

}  // namespace

PeriodicScheduler::PeriodicScheduler(double deadline_tolerance_seconds,
                                     double watchdog_seconds) {
  require_positive_finite(deadline_tolerance_seconds,
                          "deadline_tolerance_seconds");
  require_positive_finite(watchdog_seconds, "watchdog_seconds");
  if (watchdog_seconds < deadline_tolerance_seconds) {
    throw std::invalid_argument(
        "watchdog_seconds cannot be less than deadline tolerance");
  }
  deadline_tolerance_ = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(deadline_tolerance_seconds));
  watchdog_limit_ = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(watchdog_seconds));
}

ScheduleMetrics PeriodicScheduler::run(std::size_t frame_count,
                                       double duration_seconds,
                                       const PublishCallback &publish) {
  if (frame_count < 2) {
    throw std::invalid_argument("frame_count must be at least 2");
  }
  require_positive_finite(duration_seconds, "duration_seconds");
  if (!publish) {
    throw std::invalid_argument("publish callback must be set");
  }

  std::unique_lock<std::mutex> lock(run_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    throw std::runtime_error("command scheduler is already executing");
  }
  BusyGuard busy_guard(busy_);

  ScheduleMetrics metrics;
  metrics.frames_requested = frame_count;
  const auto started = std::chrono::steady_clock::now();
  const auto duration = std::chrono::duration<double>(duration_seconds);

  for (std::size_t index = 0; index < frame_count; ++index) {
    const double fraction = static_cast<double>(index) /
                            static_cast<double>(frame_count - 1);
    const auto deadline = started +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            duration * fraction);
    if (index != 0) {
      std::this_thread::sleep_until(deadline);
    }
    if (cancel_requested_.load(std::memory_order_acquire)) {
      metrics.canceled = true;
      break;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto lateness = now > deadline ? now - deadline
                                         : std::chrono::steady_clock::duration::zero();
    const auto lateness_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(lateness).count();
    metrics.max_lateness_ns = std::max(metrics.max_lateness_ns, lateness_ns);
    if (lateness > deadline_tolerance_) {
      ++metrics.deadline_misses;
    }
    if (lateness > watchdog_limit_) {
      metrics.watchdog_triggered = true;
      break;
    }

    publish(index, smoothstep_progress(index, frame_count));
    ++metrics.frames_published;
  }

  metrics.elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  return metrics;
}

void PeriodicScheduler::reset_cancel() noexcept {
  cancel_requested_.store(false, std::memory_order_release);
}

void PeriodicScheduler::cancel() noexcept {
  cancel_requested_.store(true, std::memory_order_release);
}

bool PeriodicScheduler::busy() const noexcept {
  return busy_.load(std::memory_order_acquire);
}

std::size_t smoothstep_frame_count(double duration_seconds, double rate_hz,
                                   double max_abs_delta,
                                   double max_delta_per_step) {
  require_positive_finite(duration_seconds, "duration_seconds");
  require_positive_finite(rate_hz, "rate_hz");
  require_positive_finite(max_delta_per_step, "max_delta_per_step");
  if (!std::isfinite(max_abs_delta) || max_abs_delta < 0.0) {
    throw std::invalid_argument(
        "max_abs_delta must be a non-negative finite value");
  }
  const auto by_time = static_cast<std::size_t>(
      std::ceil(duration_seconds * rate_hz)) + 1;
  const auto by_delta = static_cast<std::size_t>(
      std::ceil(1.5 * max_abs_delta / max_delta_per_step)) + 1;
  return std::max<std::size_t>(2, std::max(by_time, by_delta));
}

double smoothstep_progress(std::size_t index, std::size_t frame_count) {
  if (frame_count < 2 || index >= frame_count) {
    throw std::invalid_argument("invalid smoothstep frame index");
  }
  const double fraction = static_cast<double>(index) /
                          static_cast<double>(frame_count - 1);
  return fraction * fraction * (3.0 - 2.0 * fraction);
}

}  // namespace x2_grasp
