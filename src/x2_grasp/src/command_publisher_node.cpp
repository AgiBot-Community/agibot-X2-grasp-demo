#include "x2_grasp/action/execute_command.hpp"
#include "x2_grasp/command_scheduler.hpp"

#include <aimdk_msgs/msg/hand_command.hpp>
#include <aimdk_msgs/msg/hand_command_array.hpp>
#include <aimdk_msgs/msg/hand_type.hpp>
#include <aimdk_msgs/msg/upper_body_command_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace x2_grasp {
namespace {

constexpr std::size_t kArmJointCount = 14;
constexpr std::int8_t kGripperHandType = 2;
constexpr std::int8_t kNoHandType = 0;
constexpr char kLeftJointName[] = "left_claw_joint";
constexpr char kRightJointName[] = "right_claw_joint";

bool finite_in_range(double value, double lower, double upper) {
  return std::isfinite(value) && value >= lower && value <= upper;
}

}  // namespace

class CommandPublisherNode : public rclcpp::Node {
 public:
  using ExecuteCommand = x2_grasp::action::ExecuteCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCommand>;

  CommandPublisherNode()
      : Node("x2_command_publisher"),
        publish_rate_hz_(declare_parameter("publish_rate_hz", 50.0)),
        max_delta_per_step_(declare_parameter("max_delta_per_step", 0.03)),
        max_duration_seconds_(declare_parameter("max_duration_seconds", 30.0)),
        hold_on_stop_(declare_parameter("hold_on_stop", true)),
        scheduler_(declare_parameter("deadline_tolerance_ms", 2.0) / 1000.0,
                   declare_parameter("watchdog_ms", 60.0) / 1000.0) {
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (!std::isfinite(max_delta_per_step_) || max_delta_per_step_ <= 0.0) {
      throw std::invalid_argument("max_delta_per_step must be positive");
    }
    if (!std::isfinite(max_duration_seconds_) || max_duration_seconds_ <= 0.0) {
      throw std::invalid_argument("max_duration_seconds must be positive");
    }
    source_ = declare_parameter("source", "x2_arm");
    frame_id_ = declare_parameter("frame_id", "mc_upper_body");
    const auto arm_topic =
        declare_parameter("command_topic", "/mc/upper_body_command");
    const auto hand_topic = declare_parameter(
        "hand_command_topic", "/aima/hal/joint/hand/command");
    const auto action_name = declare_parameter(
        "execute_command_action", "/x2_grasp/execute_command");

    arm_publisher_ = create_publisher<aimdk_msgs::msg::UpperBodyCommandArray>(
        arm_topic, rclcpp::QoS(10));
    hand_publisher_ = create_publisher<aimdk_msgs::msg::HandCommandArray>(
        hand_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local());
    action_server_ = rclcpp_action::create_server<ExecuteCommand>(
        this, action_name,
        std::bind(&CommandPublisherNode::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CommandPublisherNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&CommandPublisherNode::handle_accepted, this,
                  std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
                "C++ command publisher ready: %.3f Hz, action=%s",
                publish_rate_hz_, action_name.c_str());
  }

  ~CommandPublisherNode() override {
    scheduler_.cancel();
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const ExecuteCommand::Goal> goal) {
    if (executing_.load(std::memory_order_acquire) || scheduler_.busy()) {
      RCLCPP_WARN(get_logger(), "Rejecting command while another stream is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->kind != ExecuteCommand::Goal::ARM_TRAJECTORY &&
        goal->kind != ExecuteCommand::Goal::HAND_COMMAND) {
      RCLCPP_WARN(get_logger(), "Rejecting command with unknown kind=%u", goal->kind);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle>) {
    scheduler_.cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      auto result = std::make_shared<ExecuteCommand::Result>();
      result->error = "another command stream became active";
      goal_handle->abort(result);
      return;
    }
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(
        [this, goal_handle] { execute(goal_handle); });
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<ExecuteCommand::Result>();
    try {
      scheduler_.reset_cancel();
      if (goal_handle->is_canceling()) {
        scheduler_.cancel();
      }
      const auto goal = goal_handle->get_goal();
      auto metrics = goal->kind == ExecuteCommand::Goal::ARM_TRAJECTORY
                         ? execute_arm(*goal, goal_handle)
                         : execute_hand(*goal, goal_handle);
      copy_metrics(metrics, *result);
      if (metrics.canceled || goal_handle->is_canceling()) {
        result->canceled = true;
        result->error = "command stream canceled";
        goal_handle->canceled(result);
      } else if (metrics.watchdog_triggered) {
        result->error = "command publisher watchdog deadline exceeded";
        goal_handle->abort(result);
      } else {
        result->success = true;
        goal_handle->succeed(result);
      }
    } catch (const std::exception &error) {
      result->error = error.what();
      RCLCPP_ERROR(get_logger(), "Command stream failed: %s", error.what());
      goal_handle->abort(result);
    }
    executing_.store(false, std::memory_order_release);
  }

  ScheduleMetrics execute_arm(const ExecuteCommand::Goal &goal,
                              const std::shared_ptr<GoalHandle> &goal_handle) {
    validate_arm(goal.start_arm_pos, "start_arm_pos");
    validate_arm(goal.goal_arm_pos, "goal_arm_pos");
    require_duration(goal.duration);
    double max_delta = 0.0;
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
      max_delta = std::max(
          max_delta,
          std::abs(goal.goal_arm_pos[index] - goal.start_arm_pos[index]));
    }
    const auto frames = smoothstep_frame_count(
        goal.duration, publish_rate_hz_, max_delta, max_delta_per_step_);
    std::optional<aimdk_msgs::msg::UpperBodyCommandArray> last_message;
    auto metrics = scheduler_.run(
        frames, goal.duration,
        [this, &goal, &last_message, &goal_handle](std::size_t index,
                                                  double progress) {
          aimdk_msgs::msg::UpperBodyCommandArray message;
          message.header.stamp = now();
          message.header.frame_id = frame_id_;
          message.header.sequence = sequence_.fetch_add(1);
          message.source = source_;
          message.hand_sub_mode = 1;
          message.head_pos = {0.0, 0.0};
          message.arm_pos.resize(kArmJointCount);
          for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
            message.arm_pos[joint] = goal.start_arm_pos[joint] +
                (goal.goal_arm_pos[joint] - goal.start_arm_pos[joint]) * progress;
          }
          message.hand_pos = {1.0, 1.0};
          arm_publisher_->publish(message);
          last_message = std::move(message);
          publish_feedback(goal_handle, index + 1);
        });
    publish_hold_if_needed(metrics, last_message, arm_publisher_);
    return metrics;
  }

  ScheduleMetrics execute_hand(const ExecuteCommand::Goal &goal,
                               const std::shared_ptr<GoalHandle> &goal_handle) {
    require_duration(goal.duration);
    if (goal.hand != "left" && goal.hand != "right" && goal.hand != "both") {
      throw std::invalid_argument("hand must be left, right, or both");
    }
    if ((goal.hand == "left" || goal.hand == "both") &&
        !finite_in_range(goal.left_hand_position, 0.0, 1.0)) {
      throw std::invalid_argument("left_hand_position must be in [0, 1]");
    }
    if ((goal.hand == "right" || goal.hand == "both") &&
        !finite_in_range(goal.right_hand_position, 0.0, 1.0)) {
      throw std::invalid_argument("right_hand_position must be in [0, 1]");
    }
    const auto frames = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::ceil(goal.duration * publish_rate_hz_)) + 1);
    std::optional<aimdk_msgs::msg::HandCommandArray> last_message;
    auto metrics = scheduler_.run(
        frames, goal.duration,
        [this, &goal, &last_message, &goal_handle](std::size_t index, double) {
          auto message = make_hand_message(goal);
          hand_publisher_->publish(message);
          last_message = std::move(message);
          publish_feedback(goal_handle, index + 1);
        });
    publish_hold_if_needed(metrics, last_message, hand_publisher_);
    return metrics;
  }

  aimdk_msgs::msg::HandCommandArray make_hand_message(
      const ExecuteCommand::Goal &goal) const {
    aimdk_msgs::msg::HandCommandArray message;
    const bool left = goal.hand == "left" || goal.hand == "both";
    const bool right = goal.hand == "right" || goal.hand == "both";
    message.left_hand_type.value = left ? kGripperHandType : kNoHandType;
    message.right_hand_type.value = right ? kGripperHandType : kNoHandType;
    if (left) {
      message.left_hands.push_back(
          make_hand_command(kLeftJointName, goal.left_hand_position));
    }
    if (right) {
      message.right_hands.push_back(
          make_hand_command(kRightJointName, goal.right_hand_position));
    }
    return message;
  }

  static aimdk_msgs::msg::HandCommand make_hand_command(
      const std::string &name, double position) {
    aimdk_msgs::msg::HandCommand command;
    command.name = name;
    command.position = position;
    command.velocity = 1.0;
    command.acceleration = 1.0;
    command.deceleration = 1.0;
    command.effort = 1.0;
    return command;
  }

  template <typename Message>
  void publish_hold_if_needed(
      ScheduleMetrics &metrics, const std::optional<Message> &last_message,
      const typename rclcpp::Publisher<Message>::SharedPtr &publisher) {
    if (hold_on_stop_ && last_message &&
        (metrics.canceled || metrics.watchdog_triggered)) {
      publisher->publish(*last_message);
      metrics.hold_published = true;
    }
  }

  void publish_feedback(const std::shared_ptr<GoalHandle> &goal_handle,
                        std::size_t frames_published) {
    if (frames_published != 1 && frames_published % 5 != 0) {
      return;
    }
    auto feedback = std::make_shared<ExecuteCommand::Feedback>();
    feedback->frames_published = static_cast<std::uint32_t>(frames_published);
    goal_handle->publish_feedback(feedback);
  }

  static void validate_arm(const std::vector<double> &arm, const char *name) {
    if (arm.size() != kArmJointCount) {
      throw std::invalid_argument(std::string(name) + " must contain 14 joints");
    }
    if (!std::all_of(arm.begin(), arm.end(),
                     [](double value) { return std::isfinite(value); })) {
      throw std::invalid_argument(std::string(name) +
                                  " contains a non-finite value");
    }
  }

  void require_duration(double duration) const {
    if (!std::isfinite(duration) || duration <= 0.0 ||
        duration > max_duration_seconds_) {
      throw std::invalid_argument(
          "duration must be positive, finite, and within max_duration_seconds");
    }
  }

  static void copy_metrics(const ScheduleMetrics &metrics,
                           ExecuteCommand::Result &result) {
    result.frames_requested =
        static_cast<std::uint32_t>(metrics.frames_requested);
    result.frames_published =
        static_cast<std::uint32_t>(metrics.frames_published);
    result.deadline_misses =
        static_cast<std::uint32_t>(metrics.deadline_misses);
    result.max_lateness_ns = metrics.max_lateness_ns;
    result.elapsed_seconds = metrics.elapsed_seconds;
    result.canceled = metrics.canceled;
    result.watchdog_triggered = metrics.watchdog_triggered;
    result.hold_published = metrics.hold_published;
  }

  double publish_rate_hz_;
  double max_delta_per_step_;
  double max_duration_seconds_;
  bool hold_on_stop_;
  std::string source_;
  std::string frame_id_;
  PeriodicScheduler scheduler_;
  std::atomic_bool executing_{false};
  std::atomic<std::uint64_t> sequence_{0};
  std::mutex execution_thread_mutex_;
  std::thread execution_thread_;
  rclcpp::Publisher<aimdk_msgs::msg::UpperBodyCommandArray>::SharedPtr
      arm_publisher_;
  rclcpp::Publisher<aimdk_msgs::msg::HandCommandArray>::SharedPtr
      hand_publisher_;
  rclcpp_action::Server<ExecuteCommand>::SharedPtr action_server_;
};

}  // namespace x2_grasp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<x2_grasp::CommandPublisherNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("x2_command_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
