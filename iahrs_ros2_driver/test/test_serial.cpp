#include <gtest/gtest.h>

#include <pty.h>
#include <unistd.h>

#include <string>

#include "iahrs_ros2_driver/serial.hpp"

namespace
{

class SerialTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    char slave_name[128]{};
    ASSERT_EQ(openpty(&master_fd_, &slave_fd_, slave_name, nullptr, nullptr), 0);
    slave_name_ = slave_name;
    ::close(slave_fd_);
    slave_fd_ = -1;
  }

  void TearDown() override
  {
    if (master_fd_ >= 0) {
      ::close(master_fd_);
    }
    if (slave_fd_ >= 0) {
      ::close(slave_fd_);
    }
  }

  int master_fd_{-1};
  int slave_fd_{-1};
  std::string slave_name_;
};

TEST_F(SerialTest, PreservesPartialPacketAcrossTimeout) {
  Serial serial(slave_name_, 115200);
  ASSERT_TRUE(serial.open());

  const std::string first_half = "1.0,2.0,";
  ASSERT_EQ(
      ::write(master_fd_, first_half.data(), first_half.size()),
      static_cast<ssize_t>(first_half.size()));

  std::string line;
  EXPECT_EQ(serial.read_until(line, "\r\n", 5, 128),
            SERIAL_READ_LINE_TIMEOUT);

  const std::string second_half = "3.0\r\n";
  ASSERT_EQ(
      ::write(master_fd_, second_half.data(), second_half.size()),
      static_cast<ssize_t>(second_half.size()));

  EXPECT_GT(serial.read_until(line, "\r\n", 50, 128), 0);
  EXPECT_EQ(line, first_half + second_half);
}

TEST_F(SerialTest, DropsOversizedLineAndRecoversAtBoundary) {
  Serial serial(slave_name_, 115200);
  ASSERT_TRUE(serial.open());

  const std::string input = "123456789\r\nok\r\n";
  ASSERT_EQ(
      ::write(master_fd_, input.data(), input.size()),
      static_cast<ssize_t>(input.size()));

  std::string line;
  EXPECT_EQ(serial.read_until(line, "\r\n", 50, 8),
            SERIAL_READ_LINE_OVERFLOW);
  EXPECT_GT(serial.read_until(line, "\r\n", 50, 8), 0);
  EXPECT_EQ(line, "ok\r\n");
}

TEST_F(SerialTest, DiscardsOversizedLineTailBeforeRecovering) {
  Serial serial(slave_name_, 115200);
  ASSERT_TRUE(serial.open());

  const std::string oversized_start = "123456789";
  ASSERT_EQ(
      ::write(master_fd_, oversized_start.data(), oversized_start.size()),
      static_cast<ssize_t>(oversized_start.size()));

  std::string line;
  EXPECT_EQ(serial.read_until(line, "\r\n", 50, 8),
            SERIAL_READ_LINE_OVERFLOW);

  const std::string input = "tail\r\nok\r\n";
  ASSERT_EQ(
      ::write(master_fd_, input.data(), input.size()),
      static_cast<ssize_t>(input.size()));

  EXPECT_GT(serial.read_until(line, "\r\n", 50, 8), 0);
  EXPECT_EQ(line, "ok\r\n");
}

}  // namespace
