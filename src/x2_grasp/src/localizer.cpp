#include "x2_grasp/geometry.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <x2_grasp/msg/perception_status.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace x2_rgbd_localizer {
namespace {

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using PolygonStamped = geometry_msgs::msg::PolygonStamped;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

std::string lower_ascii(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

double validate_nonnegative_duration(double seconds, const std::string & name)
{
  const long double nanoseconds = static_cast<long double>(seconds) * 1.0e9L;
  if (!std::isfinite(seconds) || seconds < 0.0 ||
    !std::isfinite(nanoseconds) ||
    nanoseconds > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument(name + " must be finite, non-negative, and representable");
  }
  return seconds;
}

std::int64_t duration_to_ns(double seconds, const std::string & name)
{
  validate_nonnegative_duration(seconds, name);
  const long double nanoseconds = static_cast<long double>(seconds) * 1.0e9L;
  return static_cast<std::int64_t>(nanoseconds);
}

std::int64_t timestamp_distance_ns(std::int64_t first, std::int64_t second)
{
  return first >= second ? first - second : second - first;
}

void validate_rotation_and_translation(
  const std::vector<double> & rotation_values,
  const std::vector<double> & translation_values)
{
  if (rotation_values.size() != 9 || translation_values.size() != 3) {
    throw std::invalid_argument(
            "software extrinsics must contain 9 rotation and 3 translation values");
  }
  cv::Matx33d rotation;
  for (int index = 0; index < 9; ++index) {
    const double value = rotation_values[static_cast<std::size_t>(index)];
    if (!std::isfinite(value)) {
      throw std::invalid_argument("software extrinsics must be finite");
    }
    rotation(index / 3, index % 3) = value;
  }
  for (const double value : translation_values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("software extrinsics must be finite");
    }
  }
  const cv::Matx33d gram = rotation.t() * rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const double expected = row == col ? 1.0 : 0.0;
      if (!std::isfinite(gram(row, col)) ||
        std::abs(gram(row, col) - expected) > 1e-6)
      {
        throw std::invalid_argument("software rotation must be orthonormal");
      }
    }
  }
  const double determinant =
    rotation(0, 0) * (rotation(1, 1) * rotation(2, 2) - rotation(1, 2) * rotation(2, 1)) -
    rotation(0, 1) * (rotation(1, 0) * rotation(2, 2) - rotation(1, 2) * rotation(2, 0)) +
    rotation(0, 2) * (rotation(1, 0) * rotation(2, 1) - rotation(1, 1) * rotation(2, 0));
  if (!std::isfinite(determinant) || std::abs(determinant - 1.0) > 1e-6) {
    throw std::invalid_argument("software rotation must have determinant +1");
  }
}

void validate_image_and_camera_info(
  const Image & image,
  const CameraInfo & info,
  const std::string & stream_name)
{
  if (image.width != info.width || image.height != info.height) {
    throw std::invalid_argument(stream_name + " image and CameraInfo dimensions differ");
  }
  if (!image.header.frame_id.empty() && !info.header.frame_id.empty() &&
    image.header.frame_id != info.header.frame_id)
  {
    throw std::invalid_argument(stream_name + " image and CameraInfo frame_id differ");
  }
}

CameraModel camera_model(const CameraInfo & message)
{
  if (message.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    message.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("CameraInfo dimensions exceed the supported image size");
  }
  CameraModel model;
  model.width = static_cast<int>(message.width);
  model.height = static_cast<int>(message.height);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      model.k(row, col) = message.k[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  model.d.assign(message.d.begin(), message.d.end());
  model.distortion_model =
    message.distortion_model.empty() ? "plumb_bob" : message.distortion_model;
  return model;
}

CameraModel rectified_camera_model(const CameraInfo & message)
{
  CameraModel model = camera_model(message);
  cv::Matx33d projection;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const double value = message.p[static_cast<std::size_t>(row * 4 + col)];
      if (!std::isfinite(value)) {
        throw std::invalid_argument("RGB CameraInfo projection matrix must be finite");
      }
      projection(row, col) = value;
    }
  }
  if (projection(0, 0) <= 0.0 || projection(1, 1) <= 0.0 ||
    std::abs(projection(2, 2)) <= 1e-12)
  {
    throw std::invalid_argument("RGB CameraInfo projection matrix is invalid");
  }
  model.k = projection;
  // Keep the coefficient count so a zero-distortion Depth CameraInfo compares
  // equal to this rectified RGB model in the automatic alignment check.
  model.d.assign(message.d.size(), 0.0);
  model.distortion_model = "plumb_bob";
  return model;
}

struct AlignedFrame {
  builtin_interfaces::msg::Time stamp;
  std::int64_t stamp_ns{0};
  std::string source_frame;
  cv::Mat depth_m;
  CameraModel rgb_camera;
};

}  // namespace

class X2RgbdLocalizer final : public rclcpp::Node {
public:
  X2RgbdLocalizer()
  : Node("x2_rgbd_localizer"),
    // Grounding can take minutes. Keep historical TF alongside the held depth
    // frame so the eventual bbox is transformed at the capture timestamp.
    tf_buffer_(this->get_clock(), tf2::durationFromSec(300.0)),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    validateParameters();

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    const auto rgb_info_topic = get_parameter("rgb_camera_info_topic").as_string();
    const auto depth_info_topic = get_parameter("depth_camera_info_topic").as_string();
    rgb_info_sub_ = create_subscription<CameraInfo>(
      rgb_info_topic, sensor_qos,
      [this](CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(info_mutex_);
        rgb_info_ = std::move(message);
      });
    depth_info_sub_ = create_subscription<CameraInfo>(
      depth_info_topic, sensor_qos,
      [this](CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(info_mutex_);
        depth_info_ = std::move(message);
      });

    rgb_sub_.subscribe(
      this, get_parameter("rgb_image_topic").as_string(),
      sensor_qos.get_rmw_qos_profile());
    depth_sub_.subscribe(
      this, get_parameter("depth_image_topic").as_string(),
      sensor_qos.get_rmw_qos_profile());
    const auto queue_size = get_parameter("sync_queue_size").as_int();
    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(static_cast<std::uint32_t>(queue_size)), rgb_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(get_parameter("sync_slop_sec").as_double()));
    sync_->registerCallback(std::bind(
      &X2RgbdLocalizer::imagePairCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    aligned_depth_pub_ = create_publisher<Image>("~/aligned_depth", sensor_qos);
    camera_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "~/target_point_camera", rclcpp::QoS(10));
    base_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "~/target_point", rclcpp::QoS(10));
    camera_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "~/target_vector_camera", rclcpp::QoS(10));
    base_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "~/target_vector", rclcpp::QoS(10));
    status_pub_ = create_publisher<x2_grasp::msg::PerceptionStatus>(
      get_parameter("status_topic").as_string(), rclcpp::QoS(10));
    bbox_sub_ = create_subscription<PolygonStamped>(
      get_parameter("bbox_topic").as_string(), rclcpp::QoS(10),
      std::bind(&X2RgbdLocalizer::bboxCallback, this, std::placeholders::_1));
    hold_frame_sub_ = create_subscription<std_msgs::msg::Header>(
      get_parameter("hold_frame_topic").as_string(), rclcpp::QoS(10),
      std::bind(&X2RgbdLocalizer::holdFrameCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Ready: waiting for X2 RGB, depth, CameraInfo and detection_bbox (native C++ node)");
  }

private:
  void validateParameters() const
  {
    const auto require_topic = [this](const char * name) {
        if (get_parameter(name).as_string().empty()) {
          throw std::invalid_argument(std::string(name) + " must not be empty");
        }
      };
    require_topic("rgb_image_topic");
    require_topic("depth_image_topic");
    require_topic("rgb_camera_info_topic");
    require_topic("depth_camera_info_topic");
    require_topic("bbox_topic");
    require_topic("hold_frame_topic");
    require_topic("status_topic");
    require_topic("target_frame");

    const std::string mode = lower_ascii(get_parameter("alignment_mode").as_string());
    if (mode != "auto" && mode != "aligned" && mode != "software") {
      throw std::invalid_argument("alignment_mode must be auto, aligned, or software");
    }
    const std::string rgb_coordinate_mode =
      lower_ascii(get_parameter("rgb_coordinate_mode").as_string());
    if (rgb_coordinate_mode != "raw" && rgb_coordinate_mode != "rectified" &&
      rgb_coordinate_mode != "auto")
    {
      throw std::invalid_argument(
              "rgb_coordinate_mode must be raw, rectified, or auto");
    }

    const auto queue_size = get_parameter("sync_queue_size").as_int();
    if (queue_size <= 0 ||
      static_cast<std::uint64_t>(queue_size) >
      static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::invalid_argument("sync_queue_size must be in [1, UINT32_MAX]");
    }
    const auto held_queue_size = get_parameter("held_frame_queue_size").as_int();
    if (held_queue_size <= 0 || held_queue_size > 100) {
      throw std::invalid_argument("held_frame_queue_size must be in [1, 100]");
    }
    validate_nonnegative_duration(
      get_parameter("sync_slop_sec").as_double(), "sync_slop_sec");
    duration_to_ns(
      get_parameter("detection_slop_sec").as_double(), "detection_slop_sec");
    validate_nonnegative_duration(
      get_parameter("tf_timeout_sec").as_double(), "tf_timeout_sec");

    const double depth_scale = get_parameter("depth_scale").as_double();
    if (!std::isfinite(depth_scale) || depth_scale < 0.0) {
      throw std::invalid_argument("depth_scale must be zero or a positive finite value");
    }
    const double min_depth = get_parameter("min_depth_m").as_double();
    const double max_depth = get_parameter("max_depth_m").as_double();
    if (!std::isfinite(min_depth) || !std::isfinite(max_depth) ||
      min_depth <= 0.0 || max_depth < min_depth)
    {
      throw std::invalid_argument(
              "depth range must satisfy 0 < min_depth_m <= max_depth_m");
    }
    const double center_fraction = get_parameter("center_fraction").as_double();
    if (!std::isfinite(center_fraction) || center_fraction <= 0.0 ||
      center_fraction > 1.0)
    {
      throw std::invalid_argument("center_fraction must be in (0, 1]");
    }
    const auto splat_radius = get_parameter("splat_radius").as_int();
    if (splat_radius < 0 ||
      static_cast<std::uint64_t>(splat_radius) >
      static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
    {
      throw std::invalid_argument("splat_radius must fit a non-negative C++ int");
    }

    if (get_parameter("software_extrinsics_set").as_bool()) {
      validate_rotation_and_translation(
        get_parameter("depth_to_rgb_rotation").as_double_array(),
        get_parameter("depth_to_rgb_translation_m").as_double_array());
    }
  }

  void declareParameters()
  {
    const std::string root = "/aima/hal/sensor/rgbd_head_front";
    declare_parameter("rgb_image_topic", root + "/rgb_image");
    declare_parameter("depth_image_topic", root + "/depth_image");
    declare_parameter("rgb_camera_info_topic", root + "/rgb_camera_info");
    declare_parameter("depth_camera_info_topic", root + "/depth_camera_info");
    declare_parameter("bbox_topic", "~/detection_bbox");
    declare_parameter("hold_frame_topic", "/x2_grasp/grounding_image_stamp");
    declare_parameter("status_topic", "/x2_rgbd_localizer/status");
    declare_parameter("target_frame", "base_link");
    declare_parameter("alignment_mode", "auto");
    declare_parameter("rgb_coordinate_mode", "rectified");
    declare_parameter("sync_queue_size", static_cast<std::int64_t>(10));
    declare_parameter("held_frame_queue_size", static_cast<std::int64_t>(8));
    declare_parameter("sync_slop_sec", 0.04);
    declare_parameter("detection_slop_sec", 0.1);
    declare_parameter("tf_timeout_sec", 0.2);
    declare_parameter("depth_scale", 0.0);
    declare_parameter("min_depth_m", 0.05);
    declare_parameter("max_depth_m", 20.0);
    declare_parameter("center_fraction", 0.2);
    declare_parameter("splat_radius", static_cast<std::int64_t>(1));
    declare_parameter("software_extrinsics_set", false);
    declare_parameter<std::vector<double>>(
      "depth_to_rgb_rotation",
      {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
      "depth_to_rgb_translation_m", {0.0, 0.0, 0.0});
  }

  std::optional<std::pair<cv::Matx33d, cv::Vec3d>> extrinsics() const
  {
    if (!get_parameter("software_extrinsics_set").as_bool()) {
      return std::nullopt;
    }
    const auto rotation_values =
      get_parameter("depth_to_rgb_rotation").as_double_array();
    const auto translation_values =
      get_parameter("depth_to_rgb_translation_m").as_double_array();
    validate_rotation_and_translation(rotation_values, translation_values);
    cv::Matx33d rotation;
    cv::Vec3d translation;
    for (int index = 0; index < 9; ++index) {
      rotation(index / 3, index % 3) = rotation_values[static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < 3; ++index) {
      translation[index] = translation_values[static_cast<std::size_t>(index)];
    }
    return std::make_pair(rotation, translation);
  }

  void imagePairCallback(
    const Image::ConstSharedPtr & rgb_message,
    const Image::ConstSharedPtr & depth_message)
  {
    CameraInfo::ConstSharedPtr rgb_info;
    CameraInfo::ConstSharedPtr depth_info;
    {
      std::lock_guard<std::mutex> lock(info_mutex_);
      rgb_info = rgb_info_;
      depth_info = depth_info_;
    }
    if (!rgb_info || !depth_info) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Image pair received before both CameraInfo messages");
      return;
    }

    try {
      validate_image_and_camera_info(*rgb_message, *rgb_info, "RGB");
      validate_image_and_camera_info(*depth_message, *depth_info, "depth");
      const CameraModel rgb_raw_camera = camera_model(*rgb_info);
      const CameraModel rgb_rectified_camera = rectified_camera_model(*rgb_info);
      const CameraModel depth_camera = camera_model(*depth_info);
      const std::string rgb_coordinate_mode =
        lower_ascii(get_parameter("rgb_coordinate_mode").as_string());
      CameraModel rgb_camera = rgb_rectified_camera;
      const bool same_dimensions =
        depth_camera.width == rgb_rectified_camera.width &&
        depth_camera.height == rgb_rectified_camera.height;
      const bool same_frame = depth_info->header.frame_id == rgb_info->header.frame_id;
      const bool raw_camera_match = same_dimensions && same_frame &&
        camera_models_match(depth_camera, rgb_raw_camera);
      const bool rectified_camera_match = same_dimensions && same_frame &&
        camera_models_match(depth_camera, rgb_rectified_camera);
      if (rgb_coordinate_mode == "raw") {
        rgb_camera = rgb_raw_camera;
      } else if (rgb_coordinate_mode == "auto") {
        if (rectified_camera_match) {
          rgb_camera = rgb_rectified_camera;
        } else if (raw_camera_match) {
          rgb_camera = rgb_raw_camera;
        }
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "RGB coordinate mode=%s (raw_match=%s, rectified_match=%s)",
        rgb_coordinate_mode.c_str(), raw_camera_match ? "true" : "false",
        rectified_camera_match ? "true" : "false");
      const auto depth_cv = cv_bridge::toCvShare(depth_message, depth_message->encoding);
      const double configured_scale = get_parameter("depth_scale").as_double();
      if (!std::isfinite(configured_scale) || configured_scale < 0.0) {
        throw std::invalid_argument(
                "depth_scale must be zero or a positive finite value");
      }
      const std::optional<double> scale = configured_scale > 0.0 ?
        std::optional<double>(configured_scale) : std::nullopt;
      const cv::Mat depth_m = depth_to_meters(
        depth_cv->image, depth_message->encoding, scale);
      if (depth_m.rows != depth_camera.height || depth_m.cols != depth_camera.width) {
        throw std::invalid_argument("depth image dimensions do not match depth CameraInfo");
      }

      const std::string mode = lower_ascii(get_parameter("alignment_mode").as_string());
      if (mode != "auto" && mode != "aligned" && mode != "software") {
        throw std::invalid_argument("alignment_mode must be auto, aligned, or software");
      }

      const bool already_aligned =
        depth_m.rows == rgb_camera.height && depth_m.cols == rgb_camera.width &&
        same_frame && camera_models_match(depth_camera, rgb_camera);

      cv::Mat aligned_depth;
      if (mode == "aligned" || (mode == "auto" && already_aligned)) {
        if (depth_m.rows != rgb_camera.height || depth_m.cols != rgb_camera.width) {
          throw std::invalid_argument(
                  "alignment_mode=aligned but RGB/depth dimensions differ");
        }
        aligned_depth = depth_m;
      } else {
        const auto transform = extrinsics();
        if (!transform.has_value()) {
          throw std::invalid_argument(
                  "RGB/depth grids differ; set depth_to_rgb_rotation and "
                  "depth_to_rgb_translation_m, or use aligned mode only if the driver "
                  "guarantees registration");
        }
        const auto splat_radius_parameter = get_parameter("splat_radius").as_int();
        if (splat_radius_parameter < 0 ||
          static_cast<std::uint64_t>(splat_radius_parameter) >
          static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
        {
          throw std::invalid_argument("splat_radius must fit a non-negative C++ int");
        }
        aligned_depth = reproject_depth_to_rgb(
          depth_m, depth_camera, rgb_camera, transform->first, transform->second,
          get_parameter("min_depth_m").as_double(),
          get_parameter("max_depth_m").as_double(),
          static_cast<int>(splat_radius_parameter));
      }

      const std::string source_frame =
        !rgb_message->header.frame_id.empty() ? rgb_message->header.frame_id :
        rgb_info->header.frame_id;
      if (source_frame.empty()) {
        throw std::invalid_argument("RGB image and CameraInfo have empty frame_id");
      }

      AlignedFrame frame;
      frame.stamp = rgb_message->header.stamp;
      frame.stamp_ns = stamp_ns(frame.stamp);
      frame.source_frame = source_frame;
      frame.depth_m = aligned_depth.clone();
      frame.rgb_camera = rgb_camera;
      std::vector<builtin_interfaces::msg::Time> fulfilled_hold_requests;
      {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        frames_.push_back(std::move(frame));
        while (frames_.size() > 30) {
          frames_.pop_front();
        }

        const auto slop_ns = duration_to_ns(
          get_parameter("detection_slop_sec").as_double(), "detection_slop_sec");
        auto pending = pending_hold_stamps_.begin();
        while (pending != pending_hold_stamps_.end()) {
          const std::int64_t requested_ns = stamp_ns(*pending);
          const auto closest = std::min_element(
            frames_.begin(), frames_.end(),
            [requested_ns](const AlignedFrame & first, const AlignedFrame & second) {
              return timestamp_distance_ns(first.stamp_ns, requested_ns) <
                     timestamp_distance_ns(second.stamp_ns, requested_ns);
            });
          if (closest != frames_.end() &&
            timestamp_distance_ns(closest->stamp_ns, requested_ns) <= slop_ns)
          {
            AlignedFrame held = *closest;
            held.depth_m = held.depth_m.clone();
            held_frames_.push_back(std::move(held));
            const auto limit = static_cast<std::size_t>(
              get_parameter("held_frame_queue_size").as_int());
            while (held_frames_.size() > limit) {
              held_frames_.pop_front();
            }
            fulfilled_hold_requests.push_back(*pending);
            pending = pending_hold_stamps_.erase(pending);
          } else {
            ++pending;
          }
        }
      }

      for (const auto & stamp : fulfilled_hold_requests) {
        RCLCPP_INFO(
          get_logger(), "Held delayed aligned RGB-D frame for grounding request: %d.%09u",
          stamp.sec, stamp.nanosec);
      }

      auto aligned_header = rgb_message->header;
      aligned_header.frame_id = source_frame;
      auto aligned_message = cv_bridge::CvImage(
        aligned_header, sensor_msgs::image_encodings::TYPE_32FC1,
        aligned_depth).toImageMsg();
      aligned_depth_pub_->publish(*aligned_message);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  AlignedFrame selectFrame(const builtin_interfaces::msg::Time & requested_stamp) const
  {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    if (frames_.empty() && held_frames_.empty()) {
      throw std::invalid_argument("no synchronized aligned RGB-D frame available");
    }
    const std::int64_t requested_ns = stamp_ns(requested_stamp);
    AlignedFrame selected = !frames_.empty() ? frames_.back() : held_frames_.back();
    if (requested_ns != 0) {
      const auto consider = [&selected, requested_ns](const AlignedFrame & candidate) {
          if (timestamp_distance_ns(candidate.stamp_ns, requested_ns) <
            timestamp_distance_ns(selected.stamp_ns, requested_ns))
          {
            selected = candidate;
          }
        };
      for (const auto & candidate : frames_) {
        consider(candidate);
      }
      for (const auto & candidate : held_frames_) {
        consider(candidate);
      }
      const auto slop_ns = duration_to_ns(
        get_parameter("detection_slop_sec").as_double(), "detection_slop_sec");
      if (timestamp_distance_ns(selected.stamp_ns, requested_ns) > slop_ns) {
        throw std::invalid_argument(
                "no aligned depth frame close enough to detection timestamp");
      }
    }
    selected.depth_m = selected.depth_m.clone();
    return selected;
  }

  void holdFrameCallback(const std_msgs::msg::Header::ConstSharedPtr & message)
  {
    try {
      AlignedFrame frame = selectFrame(message->stamp);
      std::lock_guard<std::mutex> lock(frames_mutex_);
      held_frames_.push_back(std::move(frame));
      const auto limit = static_cast<std::size_t>(
        get_parameter("held_frame_queue_size").as_int());
      while (held_frames_.size() > limit) {
        held_frames_.pop_front();
      }
      RCLCPP_INFO(
        get_logger(), "Held aligned RGB-D frame for grounding request: %d.%09u",
        message->stamp.sec, message->stamp.nanosec);
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        const std::int64_t requested_ns = stamp_ns(message->stamp);
        const bool already_pending = std::any_of(
          pending_hold_stamps_.begin(), pending_hold_stamps_.end(),
          [requested_ns](const builtin_interfaces::msg::Time & stamp) {
            return stamp_ns(stamp) == requested_ns;
          });
        if (!already_pending) {
          pending_hold_stamps_.push_back(message->stamp);
        }
        const auto limit = static_cast<std::size_t>(
          get_parameter("held_frame_queue_size").as_int());
        while (pending_hold_stamps_.size() > limit) {
          pending_hold_stamps_.pop_front();
        }
      }
      RCLCPP_WARN(
        get_logger(),
        "Grounding RGB-D frame is not synchronized yet; queued hold request: %s",
        error.what());
    }
  }

  void bboxCallback(const PolygonStamped::ConstSharedPtr & message)
  {
    if (message->polygon.points.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "detection_bbox needs top-left and bottom-right points");
      publishStatus(false, message->header.stamp,
        "detection_bbox needs top-left and bottom-right points");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Received detection_bbox: %d.%09u",
      message->header.stamp.sec, message->header.stamp.nanosec);
    try {
      const AlignedFrame frame = selectFrame(message->header.stamp);
      if (!message->header.frame_id.empty() &&
        message->header.frame_id != frame.source_frame)
      {
        throw std::invalid_argument(
                "detection_bbox frame_id does not match the RGB image frame");
      }
      const auto & first = message->polygon.points[0];
      const auto & second = message->polygon.points[1];
      const double x1 = std::min(static_cast<double>(first.x), static_cast<double>(second.x));
      const double y1 = std::min(static_cast<double>(first.y), static_cast<double>(second.y));
      const double x2 = std::max(static_cast<double>(first.x), static_cast<double>(second.x));
      const double y2 = std::max(static_cast<double>(first.y), static_cast<double>(second.y));
      const BboxDepth sample = sample_bbox_center_depth(
        frame.depth_m, x1, y1, x2, y2,
        get_parameter("center_fraction").as_double(), 2,
        get_parameter("min_depth_m").as_double(),
        get_parameter("max_depth_m").as_double());
      const cv::Vec3d camera_xyz = deproject_pixel(
        sample.center_u, sample.center_v, sample.depth_m, frame.rgb_camera);

      geometry_msgs::msg::PointStamped camera_point;
      camera_point.header.stamp = frame.stamp;
      camera_point.header.frame_id = frame.source_frame;
      camera_point.point.x = camera_xyz[0];
      camera_point.point.y = camera_xyz[1];
      camera_point.point.z = camera_xyz[2];
      camera_point_pub_->publish(camera_point);

      geometry_msgs::msg::Vector3Stamped camera_vector;
      camera_vector.header = camera_point.header;
      camera_vector.vector.x = camera_xyz[0];
      camera_vector.vector.y = camera_xyz[1];
      camera_vector.vector.z = camera_xyz[2];
      camera_vector_pub_->publish(camera_vector);

      const std::string target_frame = get_parameter("target_frame").as_string();
      const auto transform = tf_buffer_.lookupTransform(
        target_frame, frame.source_frame, rclcpp::Time(frame.stamp),
        rclcpp::Duration::from_seconds(validate_nonnegative_duration(
          get_parameter("tf_timeout_sec").as_double(), "tf_timeout_sec")));
      const cv::Vec3d translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
      const cv::Vec4d quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      const cv::Vec3d base_xyz = transform_point(camera_xyz, translation, quaternion);

      geometry_msgs::msg::PointStamped base_point;
      base_point.header.stamp = frame.stamp;
      base_point.header.frame_id = target_frame;
      base_point.point.x = base_xyz[0];
      base_point.point.y = base_xyz[1];
      base_point.point.z = base_xyz[2];
      base_point_pub_->publish(base_point);

      geometry_msgs::msg::Vector3Stamped base_vector;
      base_vector.header = base_point.header;
      // The coordinate was computed from the held aligned frame, but consumers
      // correlate it with the grounding request/detection timestamp.
      base_vector.header.stamp = message->header.stamp;
      base_vector.vector.x = base_xyz[0];
      base_vector.vector.y = base_xyz[1];
      base_vector.vector.z = base_xyz[2];
      base_vector_pub_->publish(base_vector);
      RCLCPP_INFO(
        get_logger(), "Published target vector: xyz=[%.3f, %.3f, %.3f] stamp=%d.%09u",
        base_xyz[0], base_xyz[1], base_xyz[2],
        message->header.stamp.sec, message->header.stamp.nanosec);
      publishStatus(true, message->header.stamp, "");
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "detection_bbox localization failed: %s", error.what());
      publishStatus(false, message->header.stamp, error.what());
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "detection_bbox localization failed: %s", error.what());
      publishStatus(false, message->header.stamp, error.what());
    }
  }

  void publishStatus(
    bool success, const builtin_interfaces::msg::Time & stamp,
    const std::string & error)
  {
    x2_grasp::msg::PerceptionStatus message;
    message.image_stamp = stamp;
    message.success = success;
    message.source = "rgbd";
    message.stage = "depth_localization";
    message.error = error;
    status_pub_->publish(message);
  }

  rclcpp::Subscription<CameraInfo>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<PolygonStamped>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr hold_frame_sub_;
  rclcpp::Publisher<Image>::SharedPtr aligned_depth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr camera_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr base_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr camera_vector_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr base_vector_pub_;
  rclcpp::Publisher<x2_grasp::msg::PerceptionStatus>::SharedPtr status_pub_;
  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<Synchronizer> sync_;

  mutable std::mutex info_mutex_;
  CameraInfo::ConstSharedPtr rgb_info_;
  CameraInfo::ConstSharedPtr depth_info_;
  mutable std::mutex frames_mutex_;
  std::deque<AlignedFrame> frames_;
  std::deque<AlignedFrame> held_frames_;
  std::deque<builtin_interfaces::msg::Time> pending_hold_stamps_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace x2_rgbd_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<x2_rgbd_localizer::X2RgbdLocalizer>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("x2_rgbd_localizer"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
