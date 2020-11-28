#include "cloud.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace cloud {

Cloud::VectorType ToPoints(const cv::Mat &image,
                           const std::vector<SinCos> &elevations,
                           const std::vector<SinCos> &azimuths) {
  Cloud::VectorType points;
  points.reserve(image.total());

  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      const auto &data = image.at<cv::Vec3f>(r, c);
      Point p;
      PolarToCart(p, elevations[r], azimuths[c], data[RANGE]);
      p.intensity = data[INTENSITY];
      points.push_back(p);
    }  // c
  }    // r

  return points;
}

Cloud::VectorType ToPoints(const cv::Mat &image,
                           const std::vector<SinCos> &elevations) {
  Cloud::VectorType points;
  points.reserve(image.total());

  for (int r = 0; r < image.rows; ++r) {
    const auto *const row_ptr = image.ptr<cv::Vec3f>(r);
    for (int c = 0; c < image.cols; ++c) {
      const cv::Vec3f &data = row_ptr[c];
      const auto theta = data[AZIMUTH];

      Point p;
      if (std::isnan(data[RANGE])) {
        p.x = p.y = p.z = p.intensity = kNaNF;
      } else {
        PolarToCart(p, elevations[r], {theta}, data[RANGE]);
        p.intensity = data[INTENSITY];
      }
      points.push_back(p);
    }  // c
  }    // r

  return points;
}

Cloud ToCloud(const ImageConstPtr &image_msg, const CameraInfo &cinfo_msg,
              bool high_prec) {
  Cloud cloud;
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  const auto &D = cinfo_msg.D;
  const auto elevations = PrecomputeSinCos(D.cbegin(), D.cbegin() + image.rows);

  if (high_prec) {
    cloud.points = ToPoints(image, elevations);
  } else {
    const auto azimuths = PrecomputeSinCos(
        D.cbegin() + image.rows, D.cbegin() + image.rows + image.cols);
    cloud.points = ToPoints(image, elevations, azimuths);
  }

  cloud.header = pcl_conversions::toPCL(image_msg->header);
  cloud.width = image.cols;
  cloud.height = image.rows;
  return cloud;
}

}  // namespace cloud
