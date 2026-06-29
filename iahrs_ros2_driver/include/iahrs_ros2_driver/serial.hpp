#ifndef IAHRS_ROS2_DRIVER__SERIAL_HPP_
#define IAHRS_ROS2_DRIVER__SERIAL_HPP_

#include <cstddef>
#include <string>

constexpr int SERIAL_READ_LINE_NO_DATA = 0;
constexpr int SERIAL_READ_LINE_ERROR = -1;
constexpr int SERIAL_READ_LINE_TIMEOUT = -2;
constexpr int SERIAL_READ_LINE_OVERFLOW = -3;

class Serial {
public:
  explicit Serial(std::string port, unsigned int baud_rate);
  ~Serial();

  Serial(const Serial &) = delete;
  Serial & operator=(const Serial &) = delete;

  bool open();
  void close();

  void flush();
  int write(const char *data, size_t size);
  int read_until(
    std::string & output, const std::string & delimiter, int timeout_ms,
    size_t max_line_size);

private:
  bool extract_line_(
    std::string & output, const std::string & delimiter,
    size_t max_line_size, int & result);

  std::string port_;
  unsigned int baud_rate_;
  int serial_fd_;
  std::string receive_buffer_;
  bool discarding_line_{false};
};

#endif  // IAHRS_ROS2_DRIVER__SERIAL_HPP_
