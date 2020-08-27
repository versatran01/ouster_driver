#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Geometry>

#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/OusterOS1Config.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/cloud.h"
#include "ouster_ros/os1_ros.h"

namespace ouster_ros {
namespace OS1 {

using namespace ouster::OS1;
using namespace sensor_msgs;  // Image, CameraInfo, Imu
using Point = cloud::Point;
using Cloud = cloud::Cloud;

static constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
static constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }
static constexpr float kTau = 2 * M_PI;
static constexpr auto kNaNF = std::numeric_limits<float>::quiet_NaN();
static constexpr auto kNaND = std::numeric_limits<double>::quiet_NaN();
static constexpr float kRangeFactor = 0.001f;       // mm -> m
static constexpr double kDefaultGravity = 9.81645;  // [m/s^2] in philadelphia

/// Get frequency from lidar mode
int freq_of_lidar_mode(lidar_mode mode);

/// Convert a vector of double from deg to rad
void TransformDeg2RadInPlace(std::vector<double>& vec) {
  std::transform(vec.begin(), vec.end(), vec.begin(), deg2rad);
}

/// Convert imu packet imu msg
Imu ToImu(const PacketMsg& p, const std::string& frame_id, double gravity);

/**
 * @brief The Decoder class
 * The decoder class subscribes to lidar packets and imu packets and publishes
 * image, camera info, imu messages and tf from imu to lidar.
 *
 * Transform:
 * There will be no sensor frame, just imu and lidar frame, where lidar frame
 * is rotated by 180 degrees from the original lidar frame, so that x is
 * pointing forward.
 * Imu frame is thus aligned in orientation with lidar frame, and has a
 * translational offset.
 *
 * Image:
 * Decoded lidar packets are stored in a sensor_msgs::Image. This
 * image is a 3-channel float image. The channels are [range,
 * intensity, azimuth]. The image has been de-staggered for
 * legibility. This means that each column contains measurements taken
 * at different times, but approximately the same azimuth angle.
 *
 * Camera Info:
 * Auxillary information is stored in sensor_msgs::CameraInfo. The time between
 * two measurements is in P[0]. D stores concat([beam_altitude_angles (64),
 * encoder_azimuth_angles (col), beam_azimuth_angles (64), px_offsets (64)]).
 *
 * Params:
 * See class member variables and rqt_reconfigure.
 */
class Decoder {
 public:
  explicit Decoder(const ros::NodeHandle& pnh);

  Decoder(const Decoder&) = delete;
  Decoder operator=(const Decoder&) = delete;

  void LidarPacketCb(const PacketMsg& packet_msg);
  void ImuPacketCb(const PacketMsg& packet_buf);
  void ConfigCb(OusterOS1Config& config, int level);

 private:
  /// Reset cached image, azimuths, timestamps and curr_col
  void Reset();
  /// Decode a single packet
  void DecodeAndFill(const uint8_t* const packet);
  cv::Mat DestaggerImage(const cv::Mat& image, const std::vector<int>& offsets);

  // ros
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Publisher lidar_pub_, imu_pub_;
  image_transport::CameraPublisher camera_pub_;
  image_transport::Publisher range_pub_, intensity_pub_;
  ros::Subscriber lidar_packet_sub_, imu_packet_sub_;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  dynamic_reconfigure::Server<OusterOS1Config> server_;
  OusterOS1Config config_;

  // OS1
  sensor_info info_;                  // from os1
  std::vector<uint64_t> timestamps_;  // timestamps of each col
  std::vector<double> azimuths_;      // nominal azimuth of each col (no offset)
  cv::Mat image_;                     // image to fill with packets
  int curr_col_{0};                   // tracks current column

  // params
  bool use_intensity_;           // true - intensity, false - reflectivity
  double gravity_;               // gravity magnitude m/s^2
  uint64_t firing_cycle_ns_{0};  // time ns between two measurements
  std::string lidar_frame_, imu_frame_;
};

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  // Call service to retrieve sensor information, this must be done first
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
  // dt = 1s / (freq * width)
  firing_cycle_ns_ =
      1e9 / (freq_of_lidar_mode(info_.mode) * n_cols_of_lidar_mode(info_.mode));

  // Convert all angles to rad
  TransformDeg2RadInPlace(info_.beam_altitude_angles);
  TransformDeg2RadInPlace(info_.beam_azimuth_angles);

  // frame ids and gravity
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  imu_frame_ = pnh_.param<std::string>("imu_frame", "os1_imu");
  lidar_frame_ = pnh_.param<std::string>("lidar_frame", "os1_lidar");
  use_intensity_ = pnh_.param<bool>("use_intensity", false);

  // transform
  // Original os1 lidar frame is 180 from sensor frame, this is awkward.
  // we assume sensor frame is aligned with lidar frame, thus lidar x is forward
  // and lidar y points left. This also means that imu is now aligned with lidar
  // with only a translational offset
  //               ^ x_l
  //        x_i^   | -> /
  //           |   | a /
  // y_i <-----o   |  /
  //               | /
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

  // Setup reconfigure server
  server_.setCallback(boost::bind(&Decoder::ConfigCb, this, _1, _2));
}

void Decoder::LidarPacketCb(const PacketMsg& packet_msg) {
  const auto start = ros::Time::now();

  // Once we get a packet, just going to decoede it and fill in right away
  // This will fill data into image, azimuths, timestamps and update curr_col
  DecodeAndFill(packet_msg.buf.data());

  if (curr_col_ < config_.image_width) {
    // Noop if we didn't reach the required width
    return;
  }

  // p.20 If the azimuth data block status is bad, words in the data block
  // will be set to 0x0, but timestamp, measurement id, frame id, and encoder
  // count will remain valid Thus we can always use the timestamp of the first
  // packet
  std_msgs::Header header;
  header.frame_id = lidar_frame_;
  header.stamp.fromNSec(timestamps_.front());

  // Pixel offsets used to destagger image
  const auto offsets = get_px_offset(n_cols_of_lidar_mode(info_.mode));
  const cv::Mat fixed = DestaggerImage(image_, offsets);
  const ImagePtr image_msg =
      cv_bridge::CvImage(header, "32FC3", fixed).toImageMsg();

  // Fill in camera info
  const CameraInfoPtr cinfo_msg(new CameraInfo);
  cinfo_msg->header = header;
  cinfo_msg->height = image_msg->height;
  cinfo_msg->width = image_msg->width;
  cinfo_msg->distortion_model = info_.hostname;
  cinfo_msg->K[0] = firing_cycle_ns_;  // delta time between two measurements

  // D = [altitude, azimuth, px offset]
  cinfo_msg->D = info_.beam_altitude_angles;
  cinfo_msg->D.insert(cinfo_msg->D.end(), azimuths_.cbegin(), azimuths_.cend());
  cinfo_msg->D.insert(cinfo_msg->D.end(), offsets.cbegin(), offsets.cend());
  //  cinfo_msg->D.insert(cinfo_msg->D.end(),
  //                      info_.beam_azimuth_angles.begin(),
  //                      info_.beam_azimuth_angles.end());

  // Publish on demand
  if (camera_pub_.getNumSubscribers() > 0) {
    camera_pub_.publish(image_msg, cinfo_msg);
  }

  if (lidar_pub_.getNumSubscribers() > 0) {
    const auto start = ros::Time::now();
    lidar_pub_.publish(cloud::ToCloud(image_msg, *cinfo_msg, false));
    ROS_DEBUG("%f", (ros::Time::now() - start).toSec());
  }

  if (range_pub_.getNumSubscribers() > 0 ||
      intensity_pub_.getNumSubscribers() > 0) {
    // Publish range and intensity separately
    cv::Mat sep[3];
    cv::split(fixed, sep);

    cv::Mat range = sep[cloud::RANGE];
    // should be 2, use 3 for more contrast
    range.convertTo(range, CV_8UC1, 3.0);
    range_pub_.publish(
        cv_bridge::CvImage(header, image_encodings::MONO8, range).toImageMsg());

    // use 300 for more contrast
    cv::Mat intensity = sep[cloud::INTENSITY];
    double a, b;
    cv::minMaxIdx(intensity, &a, &b);
    intensity.convertTo(intensity, CV_8UC1, 255 / (b - a), 255 * a / (a - b));
    intensity_pub_.publish(
        cv_bridge::CvImage(header, image_encodings::MONO8, intensity)
            .toImageMsg());
  }

  // Don't forget to reset
  Reset();
  ROS_DEBUG("Total time: %f", (ros::Time::now() - start).toSec());
}

void Decoder::ImuPacketCb(const PacketMsg& packet_msg) {
  const auto imu_msg = ToImu(packet_msg, imu_frame_, gravity_);
  imu_pub_.publish(imu_msg);
}

void Decoder::ConfigCb(OusterOS1Config& config, int level) {
  if (config.full_sweep) {
    config.image_width = n_cols_of_lidar_mode(info_.mode);
  } else {
    // image_width is a multiple of columns_per_buffer
    config.image_width /= columns_per_buffer;
    config.image_width *= columns_per_buffer;
  }

  ROS_INFO("Reconfigure Request: image_width: %d, full_sweep: %s",
           config.image_width,
           config.full_sweep ? "True" : "False");

  config_ = config;
  Reset();

  // Initialization
  if (level < 0) {
    // IO
    ROS_INFO("Initializing ROS subscriber/publisher...");
    imu_packet_sub_ =
        pnh_.subscribe("imu_packets", 200, &Decoder::ImuPacketCb, this);
    lidar_packet_sub_ =
        pnh_.subscribe("lidar_packets", 256, &Decoder::LidarPacketCb, this);

    imu_pub_ = pnh_.advertise<Imu>("imu", 100);
    camera_pub_ = it_.advertiseCamera("image", 10);
    lidar_pub_ = pnh_.advertise<PointCloud2>("cloud", 10);
    range_pub_ = it_.advertise("range", 1);
    intensity_pub_ = it_.advertise("intensity", 1);
    ROS_INFO("Decoder initialized");
  }
}

void Decoder::Reset() {
  curr_col_ = 0;
  image_ = cv::Mat(
      pixels_per_column, config_.image_width, CV_32FC3, cv::Scalar(kNaNF));
  azimuths_.clear();
  azimuths_.resize(config_.image_width, kNaND);
  timestamps_.clear();
  timestamps_.resize(config_.image_width, 0);
}

void Decoder::DecodeAndFill(const uint8_t* const packet_buf) {
  // Decode each azimuth block (16 per packet)
  for (int icol = 0; icol < columns_per_buffer; ++icol, ++curr_col_) {
    const uint8_t* col_buf = nth_col(icol, packet_buf);
    // const uint16_t m_id = col_measurement_id(col_buf);
    const bool valid = col_valid(col_buf) == 0xffffffff;

    // See note in `LidarPacketCb` regarding validity
    timestamps_[curr_col_] = col_timestamp(col_buf);

    // drop invalid data in case of misconfiguration
    if (!valid) {
      ROS_DEBUG("Got invalid data block");
      continue;
    }

    // Add pi to theta to compensate for the lidar frame change
    float theta0 = col_h_angle(col_buf) + M_PI;  // rad

    // make sure theta \in [0, 2pi), always >=0 so no need to check the other
    // way
    if (theta0 >= kTau) theta0 -= kTau;

    azimuths_[curr_col_] = theta0;

    // Decode each beam (64 per block)
    for (uint8_t ipx = 0; ipx < pixels_per_column; ++ipx) {
      const uint8_t* px_buf = nth_px(ipx, col_buf);
      const uint32_t range = px_range(px_buf);

      auto& v = image_.at<cv::Vec3f>(ipx, curr_col_);
      // we use reflectivity which is intensity scaled based on range
      v[cloud::RANGE] = range * kRangeFactor;
      v[cloud::INTENSITY] =
          use_intensity_ ? px_signal_photons(px_buf) : px_reflectivity(px_buf);
      v[cloud::AZIMUTH] = theta0 + info_.beam_azimuth_angles[ipx];
    }
  }
}

Imu ToImu(const PacketMsg& p, const std::string& frame_id, double gravity) {
  Imu m;
  const uint8_t* buf = p.buf.data();

  // From software manual 1.13
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

cv::Mat Decoder::DestaggerImage(const cv::Mat& image,
                                const std::vector<int>& offsets) {
  // Destagger image
  cv::Mat fixed =
      cv::Mat(image.rows, image.cols, image.type(), cv::Scalar(kNaNF));

  // This opencv forEach is sometimes slower? Possibly due to the if in the
  // lambda??

  //  fixed.forEach<cv::Vec3f>([&offsets, &image](cv::Vec3f& pix, const int*
  //  pos) {
  //    const auto offset = offsets[pos[0]];
  //    if (pos[1] < image.cols - offset) {
  //      pix = image.at<cv::Vec3f>(pos[0], pos[1] + offset);
  //    }
  //  });

  /// VERY IMPORTANT NOTE!!!
  // alphabet is a laser beam and number is measurement time
  // so a0 is the top laser at the first measurement
  // The staggered image looks like this
  // | a0 | a1 | a2 | a3 | a4 | a5 | a6 |
  // | b0 | b1 | b2 | b3 | b4 | b5 | b6 |
  // | c0 | c1 | c2 | c3 | c4 | c5 | c6 |
  // | d0 | d1 | d2 | d3 | d4 | d5 | d6 |

  // The destaggered image looks like, assuming offset [0,1,2,3]
  // | a0 | a1 | a2 | a3 | a4 | a5 | a6 |
  // | b1 | b2 | b3 | b4 | b5 | b6 | __ |
  // | c2 | c3 | c4 | c5 | c6 | __ | __ |
  // | d3 | d4 | d5 | d6 | __ | __ | __ |

  // This means on client side, when an image arrives, one can directly
  // visualize the image and it would look normal (not staggered) and spatial
  // relationship is preserved. Each column roughly corresponds to the same
  // azimuth angle. To get the actual measurement id, add corresponding offset
  // to the column. For example, the first row will add offset 0 which means
  // I(0, 0) is at time t0. If the second row has pixel offset 1, then I(1, 0)
  // is measured at time t0 + 1 * dt, as can be seen from the second plot, where
  // I(1,0) = b1, which means beam number b with measurement 1

  for (int r = 0; r < image.rows; ++r) {
    const auto offset = offsets[r];

    auto const* const row_ptr_image = image.ptr<cv::Vec3f>(r);
    auto* row_ptr_fixed = fixed.ptr<cv::Vec3f>(r);

    for (int c = 0; c < image.cols - offset; ++c) {
      row_ptr_fixed[c] = row_ptr_image[c + offset];
    }
  }

  // As a side effect, azimuths_ should now store the value of each column
  // instead of the encoder value at that time
  // In this case, since we use the first beam's azimuth angle, we should add
  // offset[0]
  const auto azimuth_offset = info_.beam_azimuth_angles[0];
  for (auto& azimuth : azimuths_) azimuth += azimuth_offset;

  return fixed;
}

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

}  // namespace OS1
}  // namespace ouster_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "os1_decoder");
  ros::NodeHandle pnh("~");

  ouster_ros::OS1::Decoder node(pnh);
  ros::spin();
}
