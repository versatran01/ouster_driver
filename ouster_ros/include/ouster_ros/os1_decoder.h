#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
  uint64_t ToHostTime(uint64_t dev_time) const;
  bool HostTimeReady() const {
    return host_time_first_ > 0 && sensor_time_first_ > 0;
  }

  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Publisher lidar_pub_, imu_pub_;
  image_transport::CameraPublisher camera_pub_;
  ros::Subscriber lidar_packet_sub_, imu_packet_sub_;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  dynamic_reconfigure::Server<OusterOS1Config> server_;
  OusterOS1Config config_;

  // OS1
  ouster::OS1::sensor_info info_;
  std::vector<PacketMsg> buffer_;

  // params
  bool use_host_time_;
  double gravity_;
  std::string lidar_frame_, imu_frame_, sensor_frame_;

  // time related
  uint64_t firing_cycle_ns_{0};
  uint64_t host_time_first_{0};
  uint64_t sensor_time_first_{0};
};

}  // namespace OS1
}  // namespace ouster_ros
