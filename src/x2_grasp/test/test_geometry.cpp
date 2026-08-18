#include "x2_grasp/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace x2_rgbd_localizer {
namespace {

CameraModel camera(
  int width = 5,
  int height = 5,
  double fx = 100.0,
  double fy = 100.0,
  double cx = 2.0,
  double cy = 2.0)
{
  CameraModel model;
  model.width = width;
  model.height = height;
  model.k = cv::Matx33d(
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0);
  return model;
}

TEST(Geometry, ConvertsRep118DepthAndInvalidValues)
{
  cv::Mat raw(2, 2, CV_16UC1);
  raw.at<std::uint16_t>(0, 0) = 1000;
  raw.at<std::uint16_t>(0, 1) = 0;
  raw.at<std::uint16_t>(1, 0) = 2500;
  raw.at<std::uint16_t>(1, 1) = 65535;
  const cv::Mat result = depth_to_meters(raw, "16UC1");
  EXPECT_FLOAT_EQ(result.at<float>(0, 0), 1.0F);
  EXPECT_TRUE(std::isnan(result.at<float>(0, 1)));
  EXPECT_FLOAT_EQ(result.at<float>(1, 0), 2.5F);
  EXPECT_FLOAT_EQ(result.at<float>(1, 1), 65.535F);
}

TEST(Geometry, ConvertsFloat64DepthAndExplicitScale)
{
  cv::Mat raw(1, 2, CV_64FC1);
  raw.at<double>(0, 0) = 1.5;
  raw.at<double>(0, 1) = 0.0;
  const cv::Mat result = depth_to_meters(raw, "32FC1");
  EXPECT_FLOAT_EQ(result.at<float>(0, 0), 1.5F);
  EXPECT_TRUE(std::isnan(result.at<float>(0, 1)));

  cv::Mat millimeters(1, 1, CV_16UC1);
  millimeters.at<std::uint16_t>(0, 0) = 2500;
  const cv::Mat scaled = depth_to_meters(millimeters, "ignored", 0.001);
  EXPECT_FLOAT_EQ(scaled.at<float>(0, 0), 2.5F);
}

TEST(Geometry, IdentityRegistrationPreservesDepth)
{
  const auto model = camera();
  cv::Mat depth(5, 5, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  depth.at<float>(2, 2) = 1.25F;
  const cv::Mat result = reproject_depth_to_rgb(
    depth, model, model, cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, 0.0));
  EXPECT_FLOAT_EQ(result.at<float>(2, 2), 1.25F);
  EXPECT_TRUE(std::isnan(result.at<float>(0, 0)));
}

TEST(Geometry, AppliesTranslationAndZBuffer)
{
  const auto model = camera(5, 5, 10.0, 10.0);
  cv::Mat depth(5, 5, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  depth.at<float>(2, 2) = 1.0F;
  const cv::Mat translated = reproject_depth_to_rgb(
    depth, model, model, cv::Matx33d::eye(), cv::Vec3d(0.1, 0.0, 0.0));
  EXPECT_FLOAT_EQ(translated.at<float>(2, 3), 1.0F);

  const auto depth_camera = camera(2, 1, 100.0, 100.0, 0.5, 0.0);
  const auto rgb_camera = camera(1, 1, 0.1, 100.0, 0.0, 0.0);
  cv::Mat collision(1, 2, CV_32FC1);
  collision.at<float>(0, 0) = 2.0F;
  collision.at<float>(0, 1) = 1.0F;
  const cv::Mat zbuffered = reproject_depth_to_rgb(
    collision, depth_camera, rgb_camera, cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, 0.0));
  EXPECT_FLOAT_EQ(zbuffered.at<float>(0, 0), 1.0F);
}

TEST(Geometry, UsesNearestEvenProjectionRounding)
{
  const auto depth_camera = camera(6, 1, 2.0, 1.0, 0.0, 0.0);
  const auto rgb_camera = camera(4, 1, 1.0, 1.0, 0.0, 0.0);
  cv::Mat depth(1, 6, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  depth.at<float>(0, 1) = 1.0F;
  depth.at<float>(0, 3) = 1.0F;
  depth.at<float>(0, 5) = 1.0F;
  const cv::Mat result = reproject_depth_to_rgb(
    depth, depth_camera, rgb_camera, cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, 0.0),
    1.0, 1.0);
  EXPECT_FLOAT_EQ(result.at<float>(0, 0), 1.0F);
  EXPECT_TRUE(std::isnan(result.at<float>(0, 1)));
  EXPECT_FLOAT_EQ(result.at<float>(0, 2), 1.0F);
  EXPECT_TRUE(std::isnan(result.at<float>(0, 3)));
}

TEST(Geometry, SupportsOpenCvDistortionModels)
{
  const auto base = camera();
  for (const auto & model_name : {std::string("plumb_bob"), std::string("equidistant")}) {
    auto model = base;
    model.distortion_model = model_name;
    model.d = model_name == "equidistant" ?
      std::vector<double>{0.01, -0.001, 0.0001, -0.00001} :
      std::vector<double>{0.01, -0.001, 0.0001, -0.0001, 0.00001};
    cv::Mat depth(5, 5, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    depth.at<float>(2, 3) = 1.0F;
    const cv::Mat result = reproject_depth_to_rgb(
      depth, model, model, cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, 0.0));
    EXPECT_FLOAT_EQ(result.at<float>(2, 3), 1.0F) << model_name;
  }
}

TEST(Geometry, ComparesTheCompleteCameraPixelModel)
{
  const auto base = camera();
  auto different_distortion = base;
  different_distortion.d = {0.01, 0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(camera_models_match(base, different_distortion));

  auto different_model = base;
  different_model.distortion_model = "equidistant";
  different_model.d = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(camera_models_match(base, different_model));
}

TEST(Geometry, KeepsZeroCoefficientFisheyeOnTheFisheyePath)
{
  auto model = camera(100, 100, 100.0, 100.0, 50.0, 50.0);
  model.distortion_model = "equidistant";
  model.d = {0.0, 0.0, 0.0, 0.0};
  const cv::Vec3d point = deproject_pixel(80.0, 50.0, 1.0, model);
  EXPECT_NEAR(point[0], std::tan(0.3) * 1.0, 1e-9);
  EXPECT_NEAR(point[1], 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(point[2], 1.0);
}

TEST(Geometry, RejectsInvalidDepthRange)
{
  const auto model = camera();
  cv::Mat depth(5, 5, CV_32FC1, cv::Scalar(1.0F));
  EXPECT_THROW(
    reproject_depth_to_rgb(
      depth, model, model, cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, 0.0),
      2.0, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    sample_bbox_center_depth(depth, 1.0, 1.0, 3.0, 3.0, 0.2, 2, 0.0, 1.0),
    std::invalid_argument);
}

TEST(Geometry, SamplesRobustCenterDepth)
{
  cv::Mat depth(20, 20, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  depth(cv::Rect(8, 8, 5, 5)).setTo(2.0F);
  depth.at<float>(10, 10) = 10.0F;
  const auto result = sample_bbox_center_depth(depth, 5.0, 5.0, 15.0, 15.0);
  EXPECT_DOUBLE_EQ(result.center_u, 10.0);
  EXPECT_DOUBLE_EQ(result.center_v, 10.0);
  EXPECT_DOUBLE_EQ(result.depth_m, 2.0);
}

TEST(Geometry, RotatesThenTranslatesPoint)
{
  const double half = std::sqrt(0.5);
  const cv::Vec3d result = transform_point(
    cv::Vec3d(1.0, 0.0, 0.0), cv::Vec3d(1.0, 2.0, 3.0),
    cv::Vec4d(0.0, 0.0, half, half));
  EXPECT_NEAR(result[0], 1.0, 1e-12);
  EXPECT_NEAR(result[1], 3.0, 1e-12);
  EXPECT_NEAR(result[2], 3.0, 1e-12);
}

}  // namespace
}  // namespace x2_rgbd_localizer
