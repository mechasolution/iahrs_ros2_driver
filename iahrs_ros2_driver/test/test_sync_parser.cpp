#include <gtest/gtest.h>

#include "iahrs_ros2_driver/iahrs_driver.hpp"

namespace
{

constexpr uint16_t SYNC_MASK =
  IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL |
  IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO |
  IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG |
  IAHRS_DRIVER_SYNC_FLAG_QUATERNION;

TEST(SyncParser, ParsesCompletePacket) {
  const std::string packet =
    "0.1,0.2,1.0,"
    "2.0,3.0,4.0,"
    "10.0,20.0,30.0,"
    "0.9,0.1,0.2,0.3\r\n";
  iahrs_driver_sync_data_t data{};

  ASSERT_TRUE(IAHRSDriver::parse_sync_data(packet, SYNC_MASK, data));
  EXPECT_DOUBLE_EQ(data.accel.z, 1.0);
  EXPECT_DOUBLE_EQ(data.gyro.y, 3.0);
  EXPECT_DOUBLE_EQ(data.mag.x, 10.0);
  EXPECT_DOUBLE_EQ(data.quaternion_angle.w, 0.9);
  EXPECT_DOUBLE_EQ(data.quaternion_angle.z, 0.3);
}

TEST(SyncParser, AcceptsProtocolTrailingComma) {
  const std::string packet =
    "0.1,0.2,1.0,2.0,3.0,4.0,10.0,20.0,30.0,"
    "0.9,0.1,0.2,0.3,\r\n";
  iahrs_driver_sync_data_t data{};

  EXPECT_TRUE(IAHRSDriver::parse_sync_data(packet, SYNC_MASK, data));
}

TEST(SyncParser, RejectsTruncatedPacketWithoutChangingOutput) {
  // This is the shape of the packet that previously repeated 0.000662
  // into every remaining field.
  const std::string packet =
    "6,0.99994,0.001348,-0.010888,0.000662\r\n";
  iahrs_driver_sync_data_t data{};
  data.accel.x = 42.0;

  EXPECT_FALSE(IAHRSDriver::parse_sync_data(packet, SYNC_MASK, data));
  EXPECT_DOUBLE_EQ(data.accel.x, 42.0);
}

TEST(SyncParser, RejectsInvalidAndNonFiniteNumbers) {
  iahrs_driver_sync_data_t data{};
  EXPECT_FALSE(IAHRSDriver::parse_sync_data(
      "0.1,broken,1.0\r\n",
      IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL, data));
  EXPECT_FALSE(IAHRSDriver::parse_sync_data(
      "0.1,nan,1.0\r\n",
      IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL, data));
  EXPECT_FALSE(IAHRSDriver::parse_sync_data(
      "0.1,inf,1.0\r\n",
      IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL, data));
}

TEST(SyncParser, RejectsUnexpectedFields) {
  iahrs_driver_sync_data_t data{};
  EXPECT_FALSE(IAHRSDriver::parse_sync_data(
      "0.1,0.2,1.0,99.0\r\n",
      IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL, data));
}

}  // namespace
