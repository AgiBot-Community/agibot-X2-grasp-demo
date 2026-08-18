#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace x2_rgbd_localizer {

struct CameraModel {
  int width{0};
  int height{0};
  cv::Matx33d k{cv::Matx33d::eye()};
  std::vector<double> d;
  std::string distortion_model{"plumb_bob"};
};

struct BboxDepth {
  double center_u{0.0};
  double center_v{0.0};
  double depth_m{0.0};
};

cv::Mat depth_to_meters(
  const cv::Mat & depth,
  const std::string & encoding,
  const std::optional<double> & scale_override = std::nullopt);

bool camera_models_match(const CameraModel & a, const CameraModel & b);

cv::Mat reproject_depth_to_rgb(
  const cv::Mat & depth_m,
  const CameraModel & depth_camera,
  const CameraModel & rgb_camera,
  const cv::Matx33d & rotation_depth_to_rgb,
  const cv::Vec3d & translation_depth_to_rgb_m,
  double min_depth_m = 0.05,
  double max_depth_m = 20.0,
  int splat_radius = 0);

cv::Vec3d deproject_pixel(
  double u,
  double v,
  double depth_m,
  const CameraModel & model);

BboxDepth sample_bbox_center_depth(
  const cv::Mat & aligned_depth_m,
  double x1,
  double y1,
  double x2,
  double y2,
  double center_fraction = 0.2,
  int minimum_radius_px = 2,
  double min_depth_m = 0.05,
  double max_depth_m = 20.0);

cv::Vec3d transform_point(
  const cv::Vec3d & point,
  const cv::Vec3d & translation,
  const cv::Vec4d & quaternion_xyzw);

}  // namespace x2_rgbd_localizer
