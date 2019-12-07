#include "ouster_ros/os1_decoder.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace ouster_ros {
namespace OS1 {

static constexpr float deg2rad(float deg) { return deg * M_PI / 180.0; }
static constexpr float rad2deg(float rad) { return rad * 180.0 / M_PI; }
static constexpr float kNaNF = std::numeric_limits<float>::quiet_NaN();
static constexpr float kNaND = std::numeric_limits<double>::quiet_NaN();
static constexpr double kTauRadD = 2 * M_PI;

using namespace ouster::OS1;
using sensor_msgs::Imu;
using sensor_msgs::PointCloud2;

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  // IO
  imu_packet_sub_ =
      pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
  lidar_packet_sub_ =
      pnh_.subscribe("lidar_packets", 2048, &Decoder::LidarPacketCb, this);

  imu_pub_ = pnh_.advertise<Imu>("imu", 100);
  camera_pub_ = it_.advertiseCamera("image", 10);
  lidar_pub_ = pnh_.advertise<PointCloud2>("cloud", 10);

  // Call service to retrieve sensor information
  OS1ConfigSrv os1_srv;
  auto client = pnh_.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
  client.waitForExistence();

  // Beam altitude angles go from top to bottom
  // Note these are all degrees
  if (client.call(os1_srv)) {
    ROS_INFO("Reading sensor info from os1 config");
    const auto& cfg = os1_srv.response;
    info_.beam_altitude_angles = cfg.beam_altitude_angles;
    info_.beam_azimuth_angles = cfg.beam_azimuth_angles;
    info_.imu_to_sensor_transform = cfg.imu_to_sensor_transform;
    info_.lidar_to_sensor_transform = cfg.lidar_to_sensor_transform;
    info_.mode = lidar_mode_of_string(cfg.lidar_mode);
    info_.hostname = cfg.hostname;
  } else {
    ROS_ERROR("Calling os1 config service failed, revert to default");
    info_.beam_altitude_angles = beam_altitude_angles;
    info_.beam_azimuth_angles = beam_azimuth_angles;
    info_.imu_to_sensor_transform = imu_to_sensor_transform;
    info_.lidar_to_sensor_transform = lidar_to_sensor_transform;
    info_.mode = MODE_1024x10;
    info_.hostname = "UNKNOWN";
  }

  // Convert all angles to rad

  ROS_INFO("Hostname: %s", info_.hostname.c_str());
  ROS_INFO("Lidar mode: %s", to_string(info_.mode).c_str());

  image_width_ = 1024 / pixels_per_column * pixels_per_column;
  image_height_ = info_.beam_altitude_angles.size();
  buffer_.reserve(image_width_);
  if (image_height_ != pixels_per_column) {
    throw std::runtime_error("Invalid number of beams");
  }
}

void Decoder::LidarPacketCb(const PacketMsg& packet_msg) {
  ROS_DEBUG_THROTTLE(1, "Lidar packet size %zu", packet_msg.buf.size());
  buffer_.push_back(packet_msg);

  const auto curr_width = buffer_.size() * pixels_per_column;

  if (curr_width < image_width_) {
    return;
  }

  // We have enough buffer decode
  ROS_DEBUG("Got enough packets %zu, ready to publish", buffer_.size());
  cv::Mat image = cv::Mat(image_height_, curr_width, CV_32FC2, cv::Scalar());
  ROS_DEBUG("Image: %d x %d", image.rows, image.cols);

  // Each packet contains 16 azimuth block so can fill 16 columns of the range
  // image
  std::vector<double> azimuths(curr_width, kNaND);

  for (int ibuf = 0; ibuf < buffer_.size(); ++ibuf) {
    const PacketMsg& packet = buffer_[ibuf];
    // Decode each packet
    const uint8_t* packet_buf = packet.buf.data();
    // Decode each azimuth block
    for (int icol = 0; icol < columns_per_buffer; ++icol) {
      const uint8_t* col_buf = nth_col(icol, packet_buf);
      const uint16_t m_id = col_measurement_id(col_buf);
      const uint16_t f_id = col_frame_id(col_buf);
      const bool valid = col_valid(col_buf) == 0xffffffff;
      // If a packet is bad (not valid) measurement_id, encoder count, range,
      // and reflectivity will also be 0

      // drop invalid data in case of misconfiguration
      if (!valid) {
        ROS_DEBUG("Got invalid data block");
        continue;
      }

      // Put in valid azimuth angle
      const int col = ibuf * columns_per_buffer + icol;
      const float theta0 = col_h_angle(col_buf);

      // Decode each beam
      for (uint8_t ipx = 0; ipx < pixels_per_column; ipx++) {
        const uint8_t* px_buf = nth_px(ipx, col_buf);
        const uint32_t range = px_range(px_buf);

        // row - ipx, col - ipack * 16 + icol
        const int row = ipx;

        cv::Vec2f& v = image.at<cv::Vec2f>(row, col);
        v[0] = range * 0.001f;
        v[1] = px_reflectivity(px_buf);
      }
    }
  }

  buffer_.clear();
}

void Decoder::ImuPacketCb(const PacketMsg& packet) {
  ROS_DEBUG_THROTTLE(1, "Imu packet size %zu", packet.buf.size());
}

}  // namespace OS1
}  // namespace ouster_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "ouster_os1_decoder");
  ros::NodeHandle pnh("~");

  ouster_ros::OS1::Decoder node(pnh);
  ros::spin();
}
