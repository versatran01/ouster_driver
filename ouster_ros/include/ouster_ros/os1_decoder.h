#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "ouster/os1.h"
#include "ouster_ros/OusterOS1Config.h"
#include "ouster_ros/PacketMsg.h"

namespace ouster_ros {
namespace OS1 {

class Decoder {
 public:
  explicit Decoder(const ros::NodeHandle& pnh);

  Decoder(const Decoder&) = delete;
  Decoder operator=(const Decoder&) = delete;

  using PacketMsg = ouster_ros::PacketMsg;

  void LidarPacketCb(const PacketMsg& packet);
  void ImuPacketCb(const PacketMsg& packet);
  void ConfigCb(OusterOS1Config& config, int level);

 private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Publisher cloud_pub_, imu_pub_;
  image_transport::CameraPublisher camera_pub_;
  ros::Subscriber lidar_packet_sub_, imu_packet_sub_;

  // TODO: switch to dynamic reconfigure
  std::vector<PacketMsg> buffer_;
  ouster::OS1::sensor_info info_;
  dynamic_reconfigure::Server<OusterOS1Config> server_;
  OusterOS1Config config_;
};

}  // namespace OS1
}  // namespace ouster_ros
