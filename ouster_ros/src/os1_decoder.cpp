#include "ouster_ros/os1_decoder.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace ouster_ros {
namespace OS1 {

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

  ROS_INFO("Hostname: %s", info_.hostname.c_str());
  ROS_INFO("Lidar mode: %s", ouster::OS1::to_string(info_.mode).c_str());

  image_width_ = 1024 / pixels_per_column * pixels_per_column;
  image_height_ = info_.beam_altitude_angles.size();
  image_channel_ = 3;
  buffer_.reserve(image_width_);
  if (image_height_ != pixels_per_column ||
      image_height_ != pixels_per_column) {
    throw std::runtime_error("Invalid number of beams");
  }
}

void Decoder::LidarPacketCb(const PacketMsg& packet) {
  ROS_DEBUG_THROTTLE(1, "Lidar packet size %zu", packet.buf.size());
  buffer_.push_back(packet);

  const auto curr_width = buffer_.size() * pixels_per_column;

  if (curr_width < image_width_) {
    return;
  }

  // We have enough buffer decode
  ROS_DEBUG("Got enough packets %zu, ready to publish", buffer_.size());
  cv::Mat image =
      cv::Mat::zeros(image_height_, curr_width, CV_32FC(image_channel_));
  ROS_DEBUG("Image: %d x %d", image.rows, image.cols);

  for (const PacketMsg& pack : buffer_) {
    // Decode packet
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
