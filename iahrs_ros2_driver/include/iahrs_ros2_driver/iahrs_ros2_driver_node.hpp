#ifndef IAHRS_ROS2_DRIVER__IAHRS_ROS2_DRIVER_NODE_HPP_
#define IAHRS_ROS2_DRIVER__IAHRS_ROS2_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "iahrs_ros2_driver_msgs/srv/reset_orientation.hpp"
#include "iahrs_ros2_driver_msgs/srv/restart.hpp"

#include "iahrs_ros2_driver/iahrs_driver.hpp"
#include "rclcpp/rclcpp.hpp"

class IAHRSDriverNode : public rclcpp::Node {
public:
  explicit IAHRSDriverNode(const std::string & node_name);

private:
  std::unique_ptr<IAHRSDriver> imu_driver_;

  rclcpp::TimerBase::SharedPtr sync_data_timer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  sensor_msgs::msg::MagneticField mag_msg_;
  rclcpp::Service<iahrs_ros2_driver_msgs::srv::Restart>::SharedPtr restart_service_;
  rclcpp::Service<iahrs_ros2_driver_msgs::srv::ResetOrientation>::SharedPtr
    reset_orientation_service_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  std::string port_;
  std::string frame_id_;
  std::string parent_frame_id_;
  bool publish_tf_{false};
  int sync_period_ms_{1000};
  bool sync_sensor_accel_{false};
  bool sync_sensor_gyro_{false};
  bool sync_sensor_mag_{false};
  bool sync_sensor_quaternion_{false};
  bool enable_filter_{false};

  bool is_sync_enabled_in_param() const;
  bool is_imu_enabled() const;
  iahrs_driver_sync_flag_t get_sync_flag_from_param() const;

  void configure_sync_();
  void publish_static_tf_();
  void configure_covariances_();

  void sync_data_callback_();

  void restart_service_callback_(
    const std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Request> request,
    std::shared_ptr<iahrs_ros2_driver_msgs::srv::Restart::Response> response);
  void reset_orientation_service_callback_(
    const std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Request> request,
    std::shared_ptr<iahrs_ros2_driver_msgs::srv::ResetOrientation::Response> response);
};

#endif  // IAHRS_ROS2_DRIVER__IAHRS_ROS2_DRIVER_NODE_HPP_
