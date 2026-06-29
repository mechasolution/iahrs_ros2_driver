#ifndef IAHRS_ROS2_DRIVER__IAHRS_DRIVER_HPP_
#define IAHRS_ROS2_DRIVER__IAHRS_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "iahrs_ros2_driver/iahrs_obj.hpp"
#include "iahrs_ros2_driver/serial.hpp"

struct axis_data_t
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct orientation_data_t
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct iahrs_driver_sync_data_t
{
  uint64_t one_ms_time{0};
  double temperature{0.0};
  axis_data_t accel;
  axis_data_t gyro;
  axis_data_t mag;
  axis_data_t accel_gravity_removed;
  axis_data_t euler_angle;
  orientation_data_t quaternion_angle;
  axis_data_t velocity;
  axis_data_t position;
  axis_data_t vibration;
};

struct iahrs_version_t
{
  struct
  {
    int major{0};
    int minor{0};
  } sw;

  struct
  {
    int major{0};
    int minor{0};
  } hw;
};

class IAHRSDriver {
public:
  explicit IAHRSDriver(const std::string & port);
  ~IAHRSDriver() = default;

  bool initialize();
  bool reboot();

  bool set_sync(uint16_t target_mask, uint16_t period_ms);
  bool set_option(bool state);
  bool reset_euler_angle();

  bool fetch_sync_data(iahrs_driver_sync_data_t & data, int timeout_ms = 100);

  static bool parse_sync_data(
    const std::string & line, uint16_t sync_mask,
    iahrs_driver_sync_data_t & data);

  const iahrs_version_t & get_version() const
  {
    return version_;
  }

  iahrs_driver_sync_flag_t get_sync_flag() const
  {
    return (iahrs_driver_sync_flag_t)sync_mask_;
  }

private:
  static constexpr size_t MAX_RESPONSE_SIZE = 2048;

  Serial serial_;
  iahrs_version_t version_;
  uint16_t sync_mask_{IAHRS_DRIVER_SYNC_FLAG_NONE};

  bool send_obj_(const std::string & obj);
  bool send_obj_(const std::string & obj, const std::string & data);
  bool write_command_(const std::string & command);
};

#endif  // IAHRS_ROS2_DRIVER__IAHRS_DRIVER_HPP_
