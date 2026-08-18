#include "x2_grasp/geometry.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace x2_rgbd_localizer {
namespace {

constexpr double kRoundingInt64Limit = 9223372036854775808.0;

bool round_to_even(double value, std::int64_t & rounded)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const double lower = std::floor(value);
  const double fraction = value - lower;
  double rounded_value = lower;
  if (fraction > 0.5) {
    rounded_value = lower + 1.0;
  } else if (fraction == 0.5) {
    if (lower <= -kRoundingInt64Limit || lower >= kRoundingInt64Limit) {
      return false;
    }
    const auto lower_integer = static_cast<std::int64_t>(lower);
    rounded_value = (lower_integer % 2 == 0) ? lower : lower + 1.0;
  }
  if (rounded_value <= -kRoundingInt64Limit ||
    rounded_value >= kRoundingInt64Limit)
  {
    return false;
  }
  rounded = static_cast<std::int64_t>(rounded_value);
  return true;
}

std::string lower_ascii(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::string normalized_distortion_model(const CameraModel & model)
{
  const std::string normalized = lower_ascii(model.distortion_model);
  return normalized.empty() ? "plumb_bob" : normalized;
}

bool approximately_equal(double first, double second)
{
  const double scale = std::max({1.0, std::abs(first), std::abs(second)});
  return std::abs(first - second) <= 1e-6 * scale;
}

bool valid_opencv_distortion_size(std::size_t size)
{
  return size == 4 || size == 5 || size == 8 || size == 12 || size == 14;
}

void validate_distortion(const CameraModel & model)
{
  const std::string distortion_model = normalized_distortion_model(model);
  if (distortion_model != "plumb_bob" &&
    distortion_model != "rational_polynomial" &&
    distortion_model != "equidistant")
  {
    throw std::invalid_argument(
            "unsupported distortion model " + model.distortion_model);
  }
  for (const double coefficient : model.d) {
    if (!std::isfinite(coefficient)) {
      throw std::invalid_argument("camera distortion must be finite");
    }
  }
  if (distortion_model == "equidistant") {
    if (model.d.size() != 4) {
      throw std::invalid_argument("equidistant distortion needs exactly four coefficients");
    }
  } else if (!model.d.empty() && !valid_opencv_distortion_size(model.d.size())) {
    throw std::invalid_argument(
            "OpenCV distortion needs 4, 5, 8, 12, or 14 coefficients");
  }
}

void validate_camera(const CameraModel & model)
{
  if (model.width <= 0 || model.height <= 0) {
    throw std::invalid_argument("camera dimensions must be positive");
  }
  if (!std::isfinite(model.k(0, 0)) || !std::isfinite(model.k(1, 1)) ||
    !std::isfinite(model.k(0, 2)) || !std::isfinite(model.k(1, 2)) ||
    model.k(0, 0) <= 0.0 || model.k(1, 1) <= 0.0)
  {
    throw std::invalid_argument("camera intrinsics are invalid");
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!std::isfinite(model.k(row, col))) {
        throw std::invalid_argument("camera intrinsics must be finite");
      }
    }
  }
  if (std::abs(model.k(0, 1)) > 1e-6 ||
    std::abs(model.k(1, 0)) > 1e-6 ||
    std::abs(model.k(2, 0)) > 1e-6 ||
    std::abs(model.k(2, 1)) > 1e-6 ||
    std::abs(model.k(2, 2) - 1.0) > 1e-6)
  {
    throw std::invalid_argument(
            "camera intrinsics must use the standard pinhole CameraInfo layout");
  }
  validate_distortion(model);
}

bool has_distortion(const CameraModel & model)
{
  const std::string distortion_model = normalized_distortion_model(model);
  if (distortion_model == "equidistant") {
    return true;
  }
  return std::any_of(
    model.d.begin(), model.d.end(),
    [](double coefficient) {return std::abs(coefficient) > 1e-12;});
}

cv::Mat distortion_matrix(const CameraModel & model, bool fisheye)
{
  validate_distortion(model);
  if (fisheye && model.d.size() != 4) {
    throw std::invalid_argument("equidistant distortion needs exactly four coefficients");
  }
  const std::size_t count = fisheye ? 4 : model.d.size();
  cv::Mat result(static_cast<int>(count), 1, CV_64F);
  for (std::size_t i = 0; i < count; ++i) {
    const double coefficient = model.d[i];
    result.at<double>(static_cast<int>(i), 0) = coefficient;
  }
  return result;
}

bool is_fisheye(const CameraModel & model)
{
  return normalized_distortion_model(model) == "equidistant";
}

std::vector<cv::Point2d> normalized_rays(
  const std::vector<cv::Point2d> & pixels,
  const CameraModel & model)
{
  validate_camera(model);
  if (pixels.empty()) {
    return {};
  }
  if (!has_distortion(model)) {
    std::vector<cv::Point2d> rays;
    rays.reserve(pixels.size());
    for (const auto & pixel : pixels) {
      rays.emplace_back(
        (pixel.x - model.k(0, 2)) / model.k(0, 0),
        (pixel.y - model.k(1, 2)) / model.k(1, 1));
    }
    return rays;
  }

  const bool fisheye = is_fisheye(model);
  const std::string distortion_model = normalized_distortion_model(model);
  if (!fisheye && distortion_model != "plumb_bob" &&
    distortion_model != "rational_polynomial")
  {
    throw std::invalid_argument(
            "unsupported distortion model " + model.distortion_model);
  }

  const cv::Mat distortion = distortion_matrix(model, fisheye);
  cv::Mat pixel_matrix(static_cast<int>(pixels.size()), 1, CV_64FC2);
  for (std::size_t index = 0; index < pixels.size(); ++index) {
    pixel_matrix.at<cv::Vec2d>(static_cast<int>(index), 0) =
      cv::Vec2d(pixels[index].x, pixels[index].y);
  }
  cv::Mat rays_matrix;
  if (fisheye) {
    cv::fisheye::undistortPoints(pixel_matrix, rays_matrix, model.k, distortion);
  } else {
    cv::undistortPoints(pixel_matrix, rays_matrix, model.k, distortion);
  }
  std::vector<cv::Point2d> rays;
  rays.reserve(pixels.size());
  for (int index = 0; index < rays_matrix.total(); ++index) {
    const cv::Vec2d ray = rays_matrix.at<cv::Vec2d>(index);
    rays.emplace_back(ray[0], ray[1]);
  }
  return rays;
}

std::vector<cv::Point2d> project_points(
  const std::vector<cv::Point3d> & points,
  const CameraModel & model)
{
  validate_camera(model);
  if (points.empty()) {
    return {};
  }
  if (!has_distortion(model)) {
    std::vector<cv::Point2d> projected;
    projected.reserve(points.size());
    for (const auto & point : points) {
      projected.emplace_back(
        model.k(0, 0) * point.x / point.z + model.k(0, 2),
        model.k(1, 1) * point.y / point.z + model.k(1, 2));
    }
    return projected;
  }

  const bool fisheye = is_fisheye(model);
  const std::string distortion_model = normalized_distortion_model(model);
  if (!fisheye && distortion_model != "plumb_bob" &&
    distortion_model != "rational_polynomial")
  {
    throw std::invalid_argument(
            "unsupported distortion model " + model.distortion_model);
  }
  const cv::Mat distortion = distortion_matrix(model, fisheye);
  const cv::Mat zero_rotation = cv::Mat::zeros(3, 1, CV_64F);
  const cv::Mat zero_translation = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat point_matrix(static_cast<int>(points.size()), 1, CV_64FC3);
  for (std::size_t index = 0; index < points.size(); ++index) {
    point_matrix.at<cv::Vec3d>(static_cast<int>(index), 0) =
      cv::Vec3d(points[index].x, points[index].y, points[index].z);
  }
  cv::Mat projected_matrix;
  if (fisheye) {
    cv::fisheye::projectPoints(
      point_matrix, projected_matrix, zero_rotation, zero_translation,
      model.k, distortion);
  } else {
    cv::projectPoints(
      point_matrix, zero_rotation, zero_translation, model.k, distortion,
      projected_matrix);
  }
  std::vector<cv::Point2d> projected;
  projected.reserve(points.size());
  for (int index = 0; index < projected_matrix.total(); ++index) {
    const cv::Vec2d pixel = projected_matrix.at<cv::Vec2d>(index);
    projected.emplace_back(pixel[0], pixel[1]);
  }
  return projected;
}

double read_depth(const cv::Mat & depth, int row, int col)
{
  switch (depth.type()) {
    case CV_16UC1:
      return static_cast<double>(depth.at<std::uint16_t>(row, col));
    case CV_32FC1:
      return static_cast<double>(depth.at<float>(row, col));
    case CV_64FC1:
      return depth.at<double>(row, col);
    default:
      throw std::invalid_argument(
              "depth image must be CV_16UC1, CV_32FC1, or CV_64FC1");
  }
}

void splat(
  cv::Mat & z_buffer,
  double projected_u,
  double projected_v,
  float z_value,
  int radius)
{
  std::int64_t center_u = 0;
  std::int64_t center_v = 0;
  if (!round_to_even(projected_u, center_u) ||
    !round_to_even(projected_v, center_v))
  {
    return;
  }

  const std::int64_t width = z_buffer.cols;
  const std::int64_t height = z_buffer.rows;
  const std::int64_t radius64 = radius;
  if (center_u < -radius64 || center_u >= width + radius64 ||
    center_v < -radius64 || center_v >= height + radius64)
  {
    return;
  }

  const std::int64_t last_u = width - 1;
  const std::int64_t last_v = height - 1;
  const std::int64_t first_u = center_u > radius64 ? center_u - radius64 : 0;
  const std::int64_t first_v = center_v > radius64 ? center_v - radius64 : 0;
  const std::int64_t last_output_u =
    center_u <= last_u - radius64 ? center_u + radius64 : last_u;
  const std::int64_t last_output_v =
    center_v <= last_v - radius64 ? center_v + radius64 : last_v;
  for (std::int64_t row = first_v; row <= last_output_v; ++row) {
    for (std::int64_t col = first_u; col <= last_output_u; ++col) {
      float & current = z_buffer.at<float>(static_cast<int>(row), static_cast<int>(col));
      if (z_value < current) {
        current = z_value;
      }
    }
  }
}

cv::Vec3d transform_xyz(
  const cv::Matx33d & rotation,
  const cv::Vec3d & translation,
  const cv::Vec3d & point)
{
  return rotation * point + translation;
}

void validate_depth_range(double min_depth_m, double max_depth_m)
{
  if (!std::isfinite(min_depth_m) || !std::isfinite(max_depth_m) ||
    min_depth_m <= 0.0 || max_depth_m < min_depth_m)
  {
    throw std::invalid_argument(
            "depth range must satisfy 0 < min_depth_m <= max_depth_m");
  }
}

void validate_rotation(const cv::Matx33d & rotation)
{
  const cv::Matx33d orthogonality = rotation.t() * rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const double expected = row == col ? 1.0 : 0.0;
      if (!std::isfinite(rotation(row, col)) ||
        !std::isfinite(orthogonality(row, col)) ||
        std::abs(orthogonality(row, col) - expected) > 1e-6)
      {
        throw std::invalid_argument("depth-to-RGB rotation must be orthonormal");
      }
    }
  }
  const double determinant =
    rotation(0, 0) * (rotation(1, 1) * rotation(2, 2) - rotation(1, 2) * rotation(2, 1)) -
    rotation(0, 1) * (rotation(1, 0) * rotation(2, 2) - rotation(1, 2) * rotation(2, 0)) +
    rotation(0, 2) * (rotation(1, 0) * rotation(2, 1) - rotation(1, 1) * rotation(2, 0));
  if (!std::isfinite(determinant) || std::abs(determinant - 1.0) > 1e-6)
  {
    throw std::invalid_argument("depth-to-RGB rotation must have determinant +1");
  }
}

}  // namespace

cv::Mat depth_to_meters(
  const cv::Mat & depth,
  const std::string & encoding,
  const std::optional<double> & scale_override)
{
  if (depth.dims != 2 || depth.channels() != 1) {
    throw std::invalid_argument("depth image must be HxW single-channel");
  }

  double scale = 0.0;
  bool default_16_bit_encoding = false;
  bool default_32_bit_encoding = false;
  if (scale_override.has_value()) {
    if (!std::isfinite(*scale_override) || *scale_override <= 0.0) {
      throw std::invalid_argument("depth scale override must be positive");
    }
    scale = *scale_override;
  } else {
    const std::string normalized_encoding = lower_ascii(encoding);
    if (normalized_encoding == "16uc1" || normalized_encoding == "mono16") {
      scale = 0.001;
      default_16_bit_encoding = true;
    } else if (normalized_encoding == "32fc1") {
      scale = 1.0;
      default_32_bit_encoding = true;
    } else {
      throw std::invalid_argument(
              "unsupported depth encoding " + encoding +
              "; expected 16UC1 or 32FC1");
    }
  }

  if (default_16_bit_encoding && depth.type() != CV_16UC1) {
    throw std::invalid_argument("16UC1/mono16 depth must be CV_16UC1");
  }
  const bool supported_numeric_type =
    depth.type() == CV_16UC1 || depth.type() == CV_32FC1 ||
    depth.type() == CV_64FC1;
  if (!supported_numeric_type) {
    throw std::invalid_argument(
            "depth must be CV_16UC1, CV_32FC1, or CV_64FC1");
  }
  if (default_32_bit_encoding &&
    depth.type() != CV_32FC1 && depth.type() != CV_64FC1)
  {
    throw std::invalid_argument("32FC1 depth must be CV_32FC1 or CV_64FC1");
  }

  cv::Mat result(depth.rows, depth.cols, CV_32FC1);
  for (int row = 0; row < depth.rows; ++row) {
    for (int col = 0; col < depth.cols; ++col) {
      const double raw = read_depth(depth, row, col);
      const double value = raw * scale;
      const float converted = static_cast<float>(value);
      result.at<float>(row, col) =
        (!std::isfinite(value) || !std::isfinite(converted) || value <= 0.0) ?
        std::numeric_limits<float>::quiet_NaN() : converted;
    }
  }
  return result;
}

bool camera_models_match(const CameraModel & a, const CameraModel & b)
{
  validate_camera(a);
  validate_camera(b);
  if (a.width != b.width || a.height != b.height) {
    return false;
  }
  if (normalized_distortion_model(a) != normalized_distortion_model(b) ||
    a.d.size() != b.d.size())
  {
    return false;
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!approximately_equal(a.k(row, col), b.k(row, col))) {
        return false;
      }
    }
  }
  for (std::size_t index = 0; index < a.d.size(); ++index) {
    if (!approximately_equal(a.d[index], b.d[index])) {
      return false;
    }
  }
  return true;
}

cv::Mat reproject_depth_to_rgb(
  const cv::Mat & depth_m,
  const CameraModel & depth_camera,
  const CameraModel & rgb_camera,
  const cv::Matx33d & rotation_depth_to_rgb,
  const cv::Vec3d & translation_depth_to_rgb_m,
  double min_depth_m,
  double max_depth_m,
  int splat_radius)
{
  validate_camera(depth_camera);
  validate_camera(rgb_camera);
  if (depth_m.dims != 2 || depth_m.channels() != 1 ||
    (depth_m.type() != CV_32FC1 && depth_m.type() != CV_64FC1) ||
    depth_m.rows != depth_camera.height || depth_m.cols != depth_camera.width)
  {
    throw std::invalid_argument(
            "depth array dimensions/type do not match depth CameraInfo");
  }
  if (splat_radius < 0) {
    throw std::invalid_argument("splat radius cannot be negative");
  }
  validate_depth_range(min_depth_m, max_depth_m);
  validate_rotation(rotation_depth_to_rgb);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!std::isfinite(rotation_depth_to_rgb(row, col))) {
        throw std::invalid_argument("depth-to-RGB extrinsics must be finite");
      }
    }
  }
  for (int index = 0; index < 3; ++index) {
    if (!std::isfinite(translation_depth_to_rgb_m[index])) {
      throw std::invalid_argument("depth-to-RGB extrinsics must be finite");
    }
  }

  cv::Mat z_buffer(
    rgb_camera.height, rgb_camera.width, CV_32FC1,
    cv::Scalar(std::numeric_limits<float>::infinity()));

  const bool depth_has_distortion = has_distortion(depth_camera);
  const bool rgb_has_distortion = has_distortion(rgb_camera);
  if (!depth_has_distortion && !rgb_has_distortion) {
    for (int row = 0; row < depth_m.rows; ++row) {
      for (int col = 0; col < depth_m.cols; ++col) {
        const double z = read_depth(depth_m, row, col);
        if (!std::isfinite(z) || !(z >= min_depth_m && z <= max_depth_m)) {
          continue;
        }
        const cv::Vec3d depth_point(
          (static_cast<double>(col) - depth_camera.k(0, 2)) /
          depth_camera.k(0, 0) * z,
          (static_cast<double>(row) - depth_camera.k(1, 2)) /
          depth_camera.k(1, 1) * z,
          z);
        const cv::Vec3d rgb_point = transform_xyz(
          rotation_depth_to_rgb, translation_depth_to_rgb_m, depth_point);
        if (!std::isfinite(rgb_point[0]) || !std::isfinite(rgb_point[1]) ||
          !std::isfinite(rgb_point[2]) || rgb_point[2] <= 0.0)
        {
          continue;
        }
        const double projected_u =
          rgb_camera.k(0, 0) * rgb_point[0] / rgb_point[2] + rgb_camera.k(0, 2);
        const double projected_v =
          rgb_camera.k(1, 1) * rgb_point[1] / rgb_point[2] + rgb_camera.k(1, 2);
        splat(
          z_buffer, projected_u, projected_v,
          static_cast<float>(rgb_point[2]), splat_radius);
      }
    }
  } else {
    std::vector<cv::Point2d> pixels;
    std::vector<double> depths;
    pixels.reserve(static_cast<std::size_t>(depth_m.rows) * depth_m.cols);
    depths.reserve(pixels.capacity());
    for (int row = 0; row < depth_m.rows; ++row) {
      for (int col = 0; col < depth_m.cols; ++col) {
        const double z = read_depth(depth_m, row, col);
        if (std::isfinite(z) && z >= min_depth_m && z <= max_depth_m) {
          pixels.emplace_back(static_cast<double>(col), static_cast<double>(row));
          depths.push_back(z);
        }
      }
    }
    if (!pixels.empty()) {
      const auto rays = normalized_rays(pixels, depth_camera);
      std::vector<cv::Point3d> rgb_points;
      rgb_points.reserve(rays.size());
      for (std::size_t index = 0; index < rays.size(); ++index) {
        const double z = depths[index];
        const cv::Vec3d depth_point(rays[index].x * z, rays[index].y * z, z);
        const cv::Vec3d rgb_point = transform_xyz(
          rotation_depth_to_rgb, translation_depth_to_rgb_m, depth_point);
        if (std::isfinite(rgb_point[0]) && std::isfinite(rgb_point[1]) &&
          std::isfinite(rgb_point[2]) && rgb_point[2] > 0.0)
        {
          rgb_points.emplace_back(rgb_point[0], rgb_point[1], rgb_point[2]);
        }
      }
      if (!rgb_points.empty()) {
        const auto projected = project_points(rgb_points, rgb_camera);
        for (std::size_t index = 0; index < projected.size(); ++index) {
          splat(
            z_buffer, projected[index].x, projected[index].y,
            static_cast<float>(rgb_points[index].z), splat_radius);
        }
      }
    }
  }

  for (int row = 0; row < z_buffer.rows; ++row) {
    for (int col = 0; col < z_buffer.cols; ++col) {
      float & value = z_buffer.at<float>(row, col);
      if (!std::isfinite(value)) {
        value = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  return z_buffer;
}

cv::Vec3d deproject_pixel(
  double u,
  double v,
  double depth_m,
  const CameraModel & model)
{
  validate_camera(model);
  if (!std::isfinite(depth_m) || depth_m <= 0.0) {
    throw std::invalid_argument("depth must be finite and positive");
  }
  const auto rays = normalized_rays({cv::Point2d(u, v)}, model);
  if (rays.empty() || !std::isfinite(rays[0].x) || !std::isfinite(rays[0].y)) {
    throw std::runtime_error("camera deprojection produced a non-finite ray");
  }
  return cv::Vec3d(rays[0].x * depth_m, rays[0].y * depth_m, depth_m);
}

BboxDepth sample_bbox_center_depth(
  const cv::Mat & aligned_depth_m,
  double x1,
  double y1,
  double x2,
  double y2,
  double center_fraction,
  int minimum_radius_px,
  double min_depth_m,
  double max_depth_m)
{
  if (aligned_depth_m.dims != 2 || aligned_depth_m.channels() != 1 ||
    (aligned_depth_m.type() != CV_32FC1 && aligned_depth_m.type() != CV_64FC1))
  {
    throw std::invalid_argument("aligned depth must be a single-channel float image");
  }
  if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) ||
    !std::isfinite(y2) || x2 <= x1 || y2 <= y1)
  {
    throw std::invalid_argument("invalid detection bounding box");
  }
  if (!std::isfinite(center_fraction) || center_fraction <= 0.0 ||
    center_fraction > 1.0)
  {
    throw std::invalid_argument("center fraction must be in (0, 1]");
  }
  if (minimum_radius_px < 0) {
    throw std::invalid_argument("minimum radius cannot be negative");
  }
  validate_depth_range(min_depth_m, max_depth_m);

  const double center_u = 0.5 * (x1 + x2);
  const double center_v = 0.5 * (y1 + y2);
  if (center_u < 0.0 || center_u >= aligned_depth_m.cols ||
    center_v < 0.0 || center_v >= aligned_depth_m.rows)
  {
    throw std::invalid_argument("detection center lies outside the RGB image");
  }

  std::int64_t rounded_u = 0;
  std::int64_t rounded_v = 0;
  if (!round_to_even(center_u, rounded_u) || !round_to_even(center_v, rounded_v)) {
    throw std::invalid_argument("detection center cannot be represented");
  }
  std::int64_t radius_x = 0;
  std::int64_t radius_y = 0;
  if (!round_to_even((x2 - x1) * center_fraction * 0.5, radius_x) ||
    !round_to_even((y2 - y1) * center_fraction * 0.5, radius_y))
  {
    throw std::invalid_argument("detection window is too large");
  }
  radius_x = std::max<std::int64_t>(minimum_radius_px, radius_x);
  radius_y = std::max<std::int64_t>(minimum_radius_px, radius_y);

  const auto left = std::max<std::int64_t>(0, rounded_u - radius_x);
  const auto right = std::min<std::int64_t>(aligned_depth_m.cols, rounded_u + radius_x + 1);
  const auto top = std::max<std::int64_t>(0, rounded_v - radius_y);
  const auto bottom = std::min<std::int64_t>(aligned_depth_m.rows, rounded_v + radius_y + 1);

  std::vector<double> samples;
  for (std::int64_t row = top; row < bottom; ++row) {
    for (std::int64_t col = left; col < right; ++col) {
      const double value = read_depth(
        aligned_depth_m, static_cast<int>(row), static_cast<int>(col));
      if (std::isfinite(value) && value >= min_depth_m && value <= max_depth_m) {
        samples.push_back(value);
      }
    }
  }
  if (samples.empty()) {
    throw std::invalid_argument("no valid depth near detection center");
  }
  std::sort(samples.begin(), samples.end());
  const std::size_t middle = samples.size() / 2;
  const double median = samples.size() % 2 == 1 ?
    samples[middle] : 0.5 * (samples[middle - 1] + samples[middle]);
  return BboxDepth{center_u, center_v, median};
}

cv::Vec3d transform_point(
  const cv::Vec3d & point,
  const cv::Vec3d & translation,
  const cv::Vec4d & quaternion_xyzw)
{
  const double norm = std::sqrt(
    quaternion_xyzw[0] * quaternion_xyzw[0] +
    quaternion_xyzw[1] * quaternion_xyzw[1] +
    quaternion_xyzw[2] * quaternion_xyzw[2] +
    quaternion_xyzw[3] * quaternion_xyzw[3]);
  if (!std::isfinite(norm) || norm <= 0.0) {
    throw std::invalid_argument("invalid quaternion");
  }
  const double x = quaternion_xyzw[0] / norm;
  const double y = quaternion_xyzw[1] / norm;
  const double z = quaternion_xyzw[2] / norm;
  const double w = quaternion_xyzw[3] / norm;
  const cv::Vec3d qvec(x, y, z);
  const cv::Vec3d rotated = point + 2.0 *
    (w * qvec.cross(point) + qvec.cross(qvec.cross(point)));
  return rotated + translation;
}

}  // namespace x2_rgbd_localizer
