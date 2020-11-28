#pragma once

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace cloud {

using sensor_msgs::CameraInfo;
using sensor_msgs::ImageConstPtr;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
static constexpr auto kNaNF = std::numeric_limits<float>::quiet_NaN();

struct SinCos {
  SinCos() = default;
  SinCos(double rad) : sin{std::sin(rad)}, cos{std::cos(rad)} {}
  double sin, cos;
};

/// Used for indexing into packet and image, (NOISE not used now)
enum Index { RANGE = 0, INTENSITY = 1, AZIMUTH = 2, NOISE = 3 };

template <typename Iter>
std::vector<SinCos> PrecomputeSinCos(Iter first, Iter last) {
  std::vector<SinCos> sc;
  sc.reserve(std::distance(first, last));
  while (first != last) sc.emplace_back(*first++);
  return sc;
}

inline void PolarToCart(Point& p, const SinCos& ev, const SinCos& az,
                        double r) {
  p.x = r * ev.cos * az.cos;
  p.y = -r * ev.cos * az.sin;
  p.z = r * ev.sin;
}

/// High precision mode
Cloud::VectorType ToPoints(const cv::Mat& image,
                           const std::vector<SinCos>& elevations);

/// Low precision mode
Cloud::VectorType ToPoints(const cv::Mat& image,
                           const std::vector<SinCos>& elevations,
                           const std::vector<SinCos>& azimuths);

Cloud ToCloud(const ImageConstPtr& image_msg, const CameraInfo& cinfo_msg,
              bool high_prec);

}  // namespace cloud
