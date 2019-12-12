#include "ouster_ros/os1_decoder.h"

#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/os1_ros.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

namespace ouster_ros {
namespace OS1 {

using namespace ouster::OS1;
using namespace sensor_msgs;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

static constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
static constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }
static constexpr auto kNaNF = std::numeric_limits<float>::quiet_NaN();
static constexpr float kRangeFactor = 0.001f;       // mm -> m
static constexpr double kDefaultGravity = 9.81645;  // [m/s^2] in philadelphia

enum Index { RANGE = 0, INTENSITY = 1, AZIMUTH = 2 };

/// Get frequency from lidar mode
int freq_of_lidar_mode(lidar_mode mode) {
  switch (mode) {
    case MODE_512x10:
    case MODE_1024x10:
    case MODE_2048x10:
      return 10;
    case MODE_512x20:
    case MODE_1024x20:
      return 20;
    default:
      throw std::invalid_argument{"freq_of_lidar_mode"};
  }
}

/// Convert a vector of double from deg to rad
void TransformDeg2RadInPlace(std::vector<double>& vec) {
  std::transform(vec.begin(), vec.end(), vec.begin(), deg2rad);
}

/// Convert image to point cloud
CloudT ToCloud(const ImageConstPtr& image_msg, const CameraInfo& cinfo_msg,
               bool organized);

/// Convert imu packet imu msg
Imu ToImu(const PacketMsg& p, const std::string& frame_id, double gravity);

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  // Setup reconfigure server
  server_.setCallback(boost::bind(&Decoder::ConfigCb, this, _1, _2));

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
    // Should we even allow this?
    ROS_WARN("Calling os1 config service failed, revert to default");
    info_.beam_altitude_angles = beam_altitude_angles;
    info_.beam_azimuth_angles = beam_azimuth_angles;
    info_.imu_to_sensor_transform = imu_to_sensor_transform;
    info_.lidar_to_sensor_transform = lidar_to_sensor_transform;
    info_.mode = MODE_1024x10;
    info_.hostname = "UNKNOWN";
  }

  // Compute firing cycle from mode, defined as the time between each firing
  firing_cycle_ns_ =
      1e9 / (freq_of_lidar_mode(info_.mode) * n_cols_of_lidar_mode(info_.mode));

  // Convert all angles to rad
  TransformDeg2RadInPlace(info_.beam_altitude_angles);
  TransformDeg2RadInPlace(info_.beam_azimuth_angles);

  // frame ids and gravity
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  imu_frame_ = pnh_.param<std::string>("imu_frame", "os1_imu");
  lidar_frame_ = pnh_.param<std::string>("lidar_frame", "os1_lidar");
  use_intensity_ = pnh_.param<bool>("use_intensity", true);

  // transform
  // we assume sensor frame is aligned with lidar frame, thus lidar x is forward
  // and lidar y points left. This also means that imu is now aligned with lidar
  // with only a translational offset
  //               ^ x_l
  //        x_i^   |
  //           |   |
  // y_i <-----o   |
  //               |
  // <-------------o
  // y_l
  // Thus the only transform we need to publish is imu to lidar, T_L_I
  // Fix lidar frame to be aligned with sensor frame
  info_.lidar_to_sensor_transform[0] = info_.lidar_to_sensor_transform[5] = 1;
  std::vector<double> imu_to_lidar_transform(16, 0);
  using Matrix4dRow = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
  const Eigen::Map<Matrix4dRow> T_S_L(info_.lidar_to_sensor_transform.data());
  const Eigen::Map<Matrix4dRow> T_S_I(info_.imu_to_sensor_transform.data());
  Eigen::Map<Matrix4dRow> T_L_I(imu_to_lidar_transform.data());
  T_L_I = T_S_L.inverse() * T_S_I;  // T_L_I = T_L_S * T_S_I
  T_L_I.topRightCorner<3, 1>().array() *= kRangeFactor;  // to meter

  ROS_INFO_STREAM("T_S_L:\n" << T_S_L);
  ROS_INFO_STREAM("T_S_I:\n" << T_S_I);
  ROS_INFO_STREAM("T_L_I:\n" << T_L_I);

  broadcaster_.sendTransform(
      transform_to_tf_msg(imu_to_lidar_transform, lidar_frame_, imu_frame_));

  ROS_INFO("Hostname: %s", info_.hostname.c_str());
  ROS_INFO("Lidar mode: %s", to_string(info_.mode).c_str());
  ROS_INFO("Lidar frame: %s", lidar_frame_.c_str());
  ROS_INFO("Imu frame: %s", imu_frame_.c_str());
  ROS_INFO("Gravity: %f m/s^2", gravity_);
  ROS_INFO("Firing cycle ns: %zd ns", firing_cycle_ns_);
  ROS_INFO("Use intensity %s", use_intensity_ ? "true" : "false");
}

void Decoder::LidarPacketCb(const PacketMsg& packet_msg) {
  buffer_.push_back(packet_msg);

  const auto curr_width = buffer_.size() * columns_per_buffer;

  if (curr_width < config_.image_width) {
    return;
  }

  // We have enough buffer decode
  ROS_DEBUG("Got enough packets %zu, ready to publish", buffer_.size());
  // [range, reflectivity, azimuth, (noise)]
  cv::Mat image = cv::Mat(pixels_per_column, curr_width, CV_32FC3,
                          cv::Scalar(kNaNF));  // Init to all NaNs
  ROS_DEBUG("Image: %d x %d x %d", image.rows, image.cols, image.channels());

  // Decode each packet
  for (int ibuf = 0; ibuf < buffer_.size(); ++ibuf) {
    const uint8_t* packet_buf = buffer_[ibuf].buf.data();

    // Decode each azimuth block (16 per packet)
    for (int icol = 0; icol < columns_per_buffer; ++icol) {
      const uint8_t* col_buf = nth_col(icol, packet_buf);
      // const uint16_t m_id = col_measurement_id(col_buf);
      const bool valid = col_valid(col_buf) == 0xffffffff;

      // drop invalid data in case of misconfiguration
      if (!valid) {
        ROS_DEBUG("Got invalid data block");
        continue;
      }

      const int col = ibuf * columns_per_buffer + icol;
      // Add pi to theta to compensate for the lidar frame change
      const float theta0 = col_h_angle(col_buf) + M_PI;  // rad

      // Decode each beam (64 per block)
      for (uint8_t ipx = 0; ipx < pixels_per_column; ipx++) {
        const uint8_t* px_buf = nth_px(ipx, col_buf);
        const uint32_t range = px_range(px_buf);

        auto& v = image.at<cv::Vec3f>(ipx, col);
        // we use reflectivity which is intensity scaled based on range
        v[RANGE] = range * kRangeFactor;
        v[INTENSITY] = use_intensity_ ? px_signal_photons(px_buf)
                                      : px_reflectivity(px_buf);
        v[AZIMUTH] = theta0 + info_.beam_azimuth_angles[ipx];
      }
    }
  }

  std_msgs::Header header;
  header.frame_id = lidar_frame_;
  // p.20 If the azimuth data block status is bad, words in the data block will
  // be set to 0x0, but timestamp, measurement id, frame id, and encoder count
  // will remain valid
  // Thus we can always use the timestamp of the first packet
  uint64_t ts = col_timestamp(buffer_.front().buf.data());
  header.stamp.fromNSec(ts);

  const ImagePtr image_msg =
      cv_bridge::CvImage(header, "32FC3", image).toImageMsg();
  const CameraInfoPtr cinfo_msg(new CameraInfo);
  cinfo_msg->header = header;
  cinfo_msg->height = image_msg->height;
  cinfo_msg->width = image_msg->width;
  cinfo_msg->D = info_.beam_altitude_angles;
  cinfo_msg->P[0] = firing_cycle_ns_;
  // TODO: cinfo_msg.D saves [altitude_angles, azimuth_angles]

  if (camera_pub_.getNumSubscribers() > 0) {
    camera_pub_.publish(image_msg, cinfo_msg);
  }

  if (lidar_pub_.getNumSubscribers() > 0) {
    lidar_pub_.publish(ToCloud(image_msg, *cinfo_msg, config_.organized));
  }

  buffer_.clear();
}

void Decoder::ImuPacketCb(const PacketMsg& packet_msg) {
  const auto imu_msg = ToImu(packet_msg, imu_frame_, gravity_);
  imu_pub_.publish(imu_msg);
}

void Decoder::ConfigCb(OusterOS1Config& config, int level) {
  // min_range should <= max_range
  config.min_range = std::min(config.min_range, config.max_range);

  // image_width is a multiple of columns_per_buffer
  config.image_width /= columns_per_buffer;
  config.image_width *= columns_per_buffer;

  ROS_INFO(
      "Reconfigure Request: min_range: %f, max_range: %f, image_width: %d, "
      "organized: %s",
      config.min_range, config.max_range, config.image_width,
      config.organized ? "True" : "False");

  config_ = config;
  buffer_.clear();
  buffer_.reserve(config_.image_width);

  // Initialization
  if (level < 0) {
    // IO
    ROS_INFO("Initializing ROS subscriber/publisher...");
    imu_packet_sub_ =
        pnh_.subscribe("imu_packets", 200, &Decoder::ImuPacketCb, this);
    lidar_packet_sub_ =
        pnh_.subscribe("lidar_packets", 2048, &Decoder::LidarPacketCb, this);

    imu_pub_ = pnh_.advertise<Imu>("imu", 100);
    camera_pub_ = it_.advertiseCamera("image", 10);
    lidar_pub_ = pnh_.advertise<PointCloud2>("cloud", 10);
    ROS_INFO("Decoder initialized");
  }
}

CloudT ToCloud(const ImageConstPtr& image_msg, const CameraInfo& cinfo_msg,
               bool organized) {
  CloudT cloud;

  const auto image = cv_bridge::toCvShare(image_msg)->image;
  const auto& altitude_angles = cinfo_msg.D;

  cloud.header = pcl_conversions::toPCL(image_msg->header);
  cloud.reserve(image.total());

  for (int r = 0; r < image.rows; ++r) {
    const auto* const row_ptr = image.ptr<cv::Vec3f>(r);
    // Because image row 0 is the highest laser point
    const auto phi = altitude_angles[r];
    const auto cos_phi = std::cos(phi);
    const auto sin_phi = std::sin(phi);

    for (int c = 0; c < image.cols; ++c) {
      const cv::Vec3f& data = row_ptr[c];

      PointT p;
      if (std::isnan(data[RANGE])) {
        if (organized) {
          p.x = p.y = p.z = p.intensity = kNaNF;
          cloud.points.push_back(p);
        }
      } else {
        // p.23 lidar range data to xyz lidar coordinate frame
        // x = d * cos(phi) * cos(theta);
        // y = d * cos(phi) * sin(theta);
        // z = d * sin(phi)
        const auto d = data[RANGE];
        const auto theta = data[AZIMUTH];
        const auto x = d * cos_phi * std::cos(theta);
        const auto y = d * cos_phi * std::sin(theta);
        const auto z = d * sin_phi;

        p.x = x;
        p.y = -y;
        p.z = z;
        p.intensity = data[INTENSITY];

        cloud.points.push_back(p);
      }
    }
  }

  if (organized) {
    cloud.width = image.cols;
    cloud.height = image.rows;
  } else {
    cloud.width = cloud.size();
    cloud.height = 1;
  }

  return cloud;
}

Imu ToImu(const PacketMsg& p, const std::string& frame_id, double gravity) {
  Imu m;
  const uint8_t* buf = p.buf.data();

  // Ouster provides timestamps for both the gyro and accelerometer in order to
  // give access to the lowest level information. In most applications it is
  // acceptable to use the average of the two timestamps.
  const auto ts = imu_gyro_ts(buf) / 2 + imu_accel_ts(buf) / 2;
  m.header.stamp.fromNSec(ts);
  m.header.frame_id = frame_id;

  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 0;

  m.linear_acceleration.x = imu_la_x(buf) * gravity;
  m.linear_acceleration.y = imu_la_y(buf) * gravity;
  m.linear_acceleration.z = imu_la_z(buf) * gravity;

  m.angular_velocity.x = imu_av_x(buf) * M_PI / 180.0;
  m.angular_velocity.y = imu_av_y(buf) * M_PI / 180.0;
  m.angular_velocity.z = imu_av_z(buf) * M_PI / 180.0;

  for (int i = 0; i < 9; i++) {
    m.orientation_covariance[i] = -1;
    m.angular_velocity_covariance[i] = 0;
    m.linear_acceleration_covariance[i] = 0;
  }

  for (int i = 0; i < 9; i += 4) {
    m.linear_acceleration_covariance[i] = 0.01;
    m.angular_velocity_covariance[i] = 6e-4;
  }

  return m;
}

}  // namespace OS1
}  // namespace ouster_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "os1_decoder");
  ros::NodeHandle pnh("~");

  ouster_ros::OS1::Decoder node(pnh);
  ros::spin();
}
