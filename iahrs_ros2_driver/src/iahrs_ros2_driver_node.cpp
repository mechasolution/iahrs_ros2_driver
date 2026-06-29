#include "iahrs_ros2_driver/iahrs_ros2_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace
{

constexpr double STANDARD_GRAVITY = 9.80665;
constexpr double DEGREES_TO_RADIANS = 0.017453292519943295;
constexpr double MICROTESLA_TO_TESLA = 1.0e-6;
constexpr double MAX_QUATERNION_NORM_ERROR = 0.1;

bool normalize_quaternion(orientation_data_t & quaternion)
{
  const double squared_norm =
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z +
    quaternion.w * quaternion.w;
  if (!std::isfinite(squared_norm)) {
    return false;
  }

  const double norm = std::sqrt(squared_norm);
  if (std::abs(norm - 1.0) > MAX_QUATERNION_NORM_ERROR) {
    return false;
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return true;
}

}  // namespace

IAHRSDriverNode::IAHRSDriverNode(const std::string & node_name)
: Node(node_name)
{
  RCLCPP_INFO(get_logger(), "iAHRS Driver");

  port_ = declare_parameter<std::string>("port", "/dev/ttyIMU");
  frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
  parent_frame_id_ =
    declare_parameter<std::string>("parent_frame_id", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", false);
  sync_period_ms_ = declare_parameter<int>("sync_period_ms", 1000);
  sync_sensor_accel_ =
    declare_parameter<bool>("sync_sensor_accel", false);
  sync_sensor_gyro_ =
    declare_parameter<bool>("sync_sensor_gyro", false);
  sync_sensor_mag_ =
    declare_parameter<bool>("sync_sensor_mag", false);
  sync_sensor_quaternion_ =
    declare_parameter<bool>("sync_sensor_quaternion", false);
  enable_filter_ = declare_parameter<bool>("enable_filter", false);

  if (sync_period_ms_ < 1 || sync_period_ms_ > 60000) {
    throw std::invalid_argument("sync_period_ms must be between 1 and 60000");
  }
  if (frame_id_.empty() || parent_frame_id_.empty()) {
    throw std::invalid_argument("frame IDs must not be empty");
  }

  RCLCPP_INFO(get_logger(), "Configuration:");
  RCLCPP_INFO(get_logger(), "\tport: \"%s\"", port_.c_str());
  RCLCPP_INFO(get_logger(), "\tframe_id: \"%s\"", frame_id_.c_str());
  RCLCPP_INFO(
      get_logger(), "\tparent_frame_id: \"%s\"", parent_frame_id_.c_str());
  RCLCPP_INFO(
      get_logger(), "\tpublish_tf: %s", publish_tf_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "\tsync_period_ms: %d", sync_period_ms_);
  RCLCPP_INFO(
      get_logger(), "\tsync_sensor_accel: %s",
      sync_sensor_accel_ ? "true" : "false");
  RCLCPP_INFO(
      get_logger(), "\tsync_sensor_gyro: %s",
      sync_sensor_gyro_ ? "true" : "false");
  RCLCPP_INFO(
      get_logger(), "\tsync_sensor_mag: %s",
      sync_sensor_mag_ ? "true" : "false");
  RCLCPP_INFO(
      get_logger(), "\tsync_sensor_quaternion: %s",
      sync_sensor_quaternion_ ? "true" : "false");
  RCLCPP_INFO(
      get_logger(), "\tenable_filter: %s",
      enable_filter_ ? "true" : "false");

  imu_driver_ = std::make_unique<IAHRSDriver>(port_);
  if (!imu_driver_->initialize()) {
    throw std::runtime_error("Failed to connect IMU port");
  }
  if (!imu_driver_->reboot()) {
    throw std::runtime_error("Failed to initialize IMU");
  }

  const auto & version = imu_driver_->get_version();
  RCLCPP_INFO(
      get_logger(), "iAHRS H/W Version: %d.%d",
      version.hw.major, version.hw.minor);
  RCLCPP_INFO(
      get_logger(), "iAHRS S/W Version: %d.%d",
      version.sw.major, version.sw.minor);

  if (!imu_driver_->set_option(enable_filter_)) {
    throw std::runtime_error("Failed to configure IMU filter");
  }

  const auto qos = rclcpp::SensorDataQoS();
  imu_msg_.header.frame_id = frame_id_;
  mag_msg_.header.frame_id = frame_id_;
  configure_covariances_();
  imu_publisher_ =
    create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
  mag_publisher_ =
    create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);

  restart_service_ =
    create_service<iahrs_ros2_driver_msgs::srv::Restart>(
          "imu/restart",
    [this](
      const std::shared_ptr<
        iahrs_ros2_driver_msgs::srv::Restart::Request> request,
      std::shared_ptr<
        iahrs_ros2_driver_msgs::srv::Restart::Response> response) {
      restart_service_callback_(request, response);
          });

  reset_orientation_service_ =
    create_service<iahrs_ros2_driver_msgs::srv::ResetOrientation>(
          "imu/reset_heading",
    [this](
      const std::shared_ptr<
        iahrs_ros2_driver_msgs::srv::ResetOrientation::Request>
      request,
      std::shared_ptr<
        iahrs_ros2_driver_msgs::srv::ResetOrientation::Response>
      response) {
      reset_orientation_service_callback_(request, response);
          });

  if (is_sync_enabled_in_param()) {
    configure_sync_();
  } else {
    RCLCPP_WARN(get_logger(), "No synchronized sensor data is enabled");
  }
  if (publish_tf_) {
    publish_static_tf_();
  }

  RCLCPP_INFO(get_logger(), "iAHRS Driver Node has started");
}

bool IAHRSDriverNode::is_sync_enabled_in_param() const
{
  return is_imu_enabled() || sync_sensor_mag_;
}

bool IAHRSDriverNode::is_imu_enabled() const
{
  return sync_sensor_accel_ ||
         sync_sensor_gyro_ ||
         sync_sensor_quaternion_;
}

iahrs_driver_sync_flag_t
IAHRSDriverNode::get_sync_flag_from_param() const
{
  uint16_t mask = IAHRS_DRIVER_SYNC_FLAG_NONE;
  if (sync_sensor_accel_) {
    mask |= IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL;
  }
  if (sync_sensor_gyro_) {
    mask |= IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO;
  }
  if (sync_sensor_mag_) {
    mask |= IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG;
  }
  if (sync_sensor_quaternion_) {
    mask |= IAHRS_DRIVER_SYNC_FLAG_QUATERNION;
  }
  return static_cast<iahrs_driver_sync_flag_t>(mask);
}

void IAHRSDriverNode::configure_sync_()
{
  const auto mask = get_sync_flag_from_param();
  if (!imu_driver_->set_sync(mask, static_cast<uint16_t>(sync_period_ms_))) {
    throw std::runtime_error("Failed to start synchronized IMU data");
  }

  if (!sync_data_timer_) {
    const int polling_period_ms =
      std::max(1, std::min(sync_period_ms_, 10));
    sync_data_timer_ = create_wall_timer(
        std::chrono::milliseconds(polling_period_ms),
      [this]() {sync_data_callback_();});
  }
}

void IAHRSDriverNode::configure_covariances_()
{
  if (!sync_sensor_quaternion_) {
    imu_msg_.orientation_covariance[0] = -1.0;
  }
  if (!sync_sensor_gyro_) {
    imu_msg_.angular_velocity_covariance[0] = -1.0;
  }
  if (!sync_sensor_accel_) {
    imu_msg_.linear_acceleration_covariance[0] = -1.0;
  }
}

void IAHRSDriverNode::publish_static_tf_()
{
  tf_broadcaster_ =
    std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = get_clock()->now();
  transform.header.frame_id = parent_frame_id_;
  transform.child_frame_id = frame_id_;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

void IAHRSDriverNode::sync_data_callback_()
{
  iahrs_driver_sync_data_t data{};
  if (!imu_driver_->fetch_sync_data(data, 1)) {
    return;
  }

  if (sync_sensor_quaternion_ &&
    !normalize_quaternion(data.quaternion_angle))
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Discarding IMU packet with an invalid quaternion");
    return;
  }

  const auto stamp = get_clock()->now();
  if (is_imu_enabled()) {
    if (sync_sensor_accel_) {
      imu_msg_.linear_acceleration.x =
        data.accel.x * STANDARD_GRAVITY;
      imu_msg_.linear_acceleration.y =
        data.accel.y * STANDARD_GRAVITY;
      imu_msg_.linear_acceleration.z =
        data.accel.z * STANDARD_GRAVITY;
    }
    if (sync_sensor_gyro_) {
      imu_msg_.angular_velocity.x =
        data.gyro.x * DEGREES_TO_RADIANS;
      imu_msg_.angular_velocity.y =
        data.gyro.y * DEGREES_TO_RADIANS;
      imu_msg_.angular_velocity.z =
        data.gyro.z * DEGREES_TO_RADIANS;
    }
    if (sync_sensor_quaternion_) {
      imu_msg_.orientation.x = data.quaternion_angle.x;
      imu_msg_.orientation.y = data.quaternion_angle.y;
      imu_msg_.orientation.z = data.quaternion_angle.z;
      imu_msg_.orientation.w = data.quaternion_angle.w;
    }
    imu_msg_.header.stamp = stamp;
    imu_publisher_->publish(imu_msg_);
  }

  if (sync_sensor_mag_) {
    mag_msg_.magnetic_field.x = data.mag.x * MICROTESLA_TO_TESLA;
    mag_msg_.magnetic_field.y = data.mag.y * MICROTESLA_TO_TESLA;
    mag_msg_.magnetic_field.z = data.mag.z * MICROTESLA_TO_TESLA;
    mag_msg_.header.stamp = stamp;
    mag_publisher_->publish(mag_msg_);
  }
}

void IAHRSDriverNode::restart_service_callback_(
  const std::shared_ptr<
    iahrs_ros2_driver_msgs::srv::Restart::Request> request,
  std::shared_ptr<
    iahrs_ros2_driver_msgs::srv::Restart::Response> response)
{
  (void)request;

  bool success = imu_driver_->reboot();
  if (success) {
    success = imu_driver_->set_option(enable_filter_);
  }
  if (success && is_sync_enabled_in_param()) {
    try {
      configure_sync_();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
      success = false;
    }
  }

  response->result = success;
  if (success) {
    RCLCPP_INFO(get_logger(), "IMU restarted");
  } else {
    RCLCPP_WARN(get_logger(), "IMU restart failed");
  }
}

void IAHRSDriverNode::reset_orientation_service_callback_(
  const std::shared_ptr<
    iahrs_ros2_driver_msgs::srv::ResetOrientation::Request> request,
  std::shared_ptr<
    iahrs_ros2_driver_msgs::srv::ResetOrientation::Response> response)
{
  (void)request;

  response->result = imu_driver_->reset_euler_angle();
  if (response->result) {
    RCLCPP_INFO(get_logger(), "IMU heading reset");
  } else {
    RCLCPP_WARN(get_logger(), "IMU heading reset failed");
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(
        std::make_shared<IAHRSDriverNode>("iahrs_driver_node"));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
        rclcpp::get_logger("iahrs_driver_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
