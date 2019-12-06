#include "ouster_ros/decoder.h"

namespace ouster_ros {
namespace OS1 {

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  lidar_packet_sub_ =
      pnh_.subscribe("lidar_packets", 2048, &Decoder::LidarPacketCb, this);
  imu_packet_sub_ =
      pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
}

void Decoder::LidarPacketCb(const PacketMsg& packet) {
  // size is 12609
  // Lidar data packets consist of 16 azimuth block and are always 12608 Bytes
  // in length.
  ROS_INFO("Lidar %zu", packet.buf.size());
}

void Decoder::ImuPacketCb(const PacketMsg& packet) {
  // size is 49
  ROS_INFO("Imu %zu", packet.buf.size());
}

}  // namespace OS1
}  // namespace ouster_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "ouster_os1_decoder");
  ros::NodeHandle pnh("~");

  ouster_ros::OS1::Decoder node(pnh);
  ros::spin();
}
