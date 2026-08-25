#include "x2_grasp/command_scheduler.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

TEST(CommandScheduler, MatchesPythonSmoothstepFramePolicy) {
  EXPECT_EQ(x2_grasp::smoothstep_frame_count(2.0, 50.0, 0.2, 0.03), 101u);
  EXPECT_EQ(x2_grasp::smoothstep_frame_count(0.02, 50.0, 1.0, 0.03), 51u);
  EXPECT_DOUBLE_EQ(x2_grasp::smoothstep_progress(0, 3), 0.0);
  EXPECT_DOUBLE_EQ(x2_grasp::smoothstep_progress(1, 3), 0.5);
  EXPECT_DOUBLE_EQ(x2_grasp::smoothstep_progress(2, 3), 1.0);
}

TEST(CommandScheduler, UsesAbsoluteDeadlinesAndPublishesEndpoints) {
  x2_grasp::PeriodicScheduler scheduler(0.005, 0.050);
  std::vector<double> progress;

  const auto metrics = scheduler.run(
      3, 0.02,
      [&progress](std::size_t, double value) { progress.push_back(value); });

  ASSERT_EQ(progress.size(), 3u);
  EXPECT_DOUBLE_EQ(progress.front(), 0.0);
  EXPECT_DOUBLE_EQ(progress.back(), 1.0);
  EXPECT_EQ(metrics.frames_requested, 3u);
  EXPECT_EQ(metrics.frames_published, 3u);
  EXPECT_FALSE(metrics.canceled);
  EXPECT_FALSE(metrics.watchdog_triggered);
  EXPECT_GE(metrics.elapsed_seconds, 0.018);
}

TEST(CommandScheduler, StopsAfterCancellation) {
  x2_grasp::PeriodicScheduler scheduler(0.005, 0.050);

  const auto metrics = scheduler.run(
      5, 0.04, [&scheduler](std::size_t index, double) {
        if (index == 1) {
          scheduler.cancel();
        }
      });

  EXPECT_TRUE(metrics.canceled);
  EXPECT_EQ(metrics.frames_published, 2u);
}

TEST(CommandScheduler, TripsWatchdogAfterAStalledPublisher) {
  x2_grasp::PeriodicScheduler scheduler(0.001, 0.004);

  const auto metrics = scheduler.run(4, 0.006, [](std::size_t index, double) {
    if (index == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  EXPECT_TRUE(metrics.watchdog_triggered);
  EXPECT_EQ(metrics.frames_published, 1u);
  EXPECT_GE(metrics.deadline_misses, 1u);
}

}  // namespace
