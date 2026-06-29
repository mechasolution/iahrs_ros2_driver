#include "iahrs_ros2_driver/serial.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace
{

speed_t to_termios_baud(unsigned int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return 0;
  }
}

}  // namespace

Serial::Serial(std::string port, unsigned int baud_rate)
: port_(std::move(port)), baud_rate_(baud_rate), serial_fd_(-1) {}

Serial::~Serial()
{
  close();
}

bool Serial::open()
{
  close();

  const speed_t speed = to_termios_baud(baud_rate_);
  if (speed == 0) {
    return false;
  }

  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    return false;
  }

  termios tio{};
  if (tcgetattr(serial_fd_, &tio) < 0) {
    close();
    return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;

  if (cfsetispeed(&tio, speed) != 0 ||
    cfsetospeed(&tio, speed) != 0 ||
    tcsetattr(serial_fd_, TCSANOW, &tio) != 0)
  {
    close();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  receive_buffer_.clear();
  discarding_line_ = false;
  return true;
}

void Serial::close()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  receive_buffer_.clear();
  discarding_line_ = false;
}

void Serial::flush()
{
  if (serial_fd_ >= 0) {
    tcflush(serial_fd_, TCIOFLUSH);
  }
  receive_buffer_.clear();
  discarding_line_ = false;
}

int Serial::write(const char *data, size_t size)
{
  if (serial_fd_ < 0 || data == nullptr) {
    return SERIAL_READ_LINE_ERROR;
  }

  size_t total_written = 0;
  while (total_written < size) {
    const ssize_t written =
      ::write(serial_fd_, data + total_written, size - total_written);
    if (written > 0) {
      total_written += static_cast<size_t>(written);
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
      errno != EINTR)
    {
      return SERIAL_READ_LINE_ERROR;
    }

    pollfd descriptor{serial_fd_, POLLOUT, 0};
    const int poll_result = ::poll(&descriptor, 1, 100);
    if (poll_result <= 0) {
      return SERIAL_READ_LINE_ERROR;
    }
  }

  return static_cast<int>(total_written);
}

bool Serial::extract_line_(
  std::string & output, const std::string & delimiter,
  size_t max_line_size, int & result)
{
  if (discarding_line_) {
    const size_t delimiter_position = receive_buffer_.find(delimiter);
    if (delimiter_position == std::string::npos) {
      receive_buffer_.clear();
      return false;
    }
    receive_buffer_.erase(0, delimiter_position + delimiter.size());
    discarding_line_ = false;
  }

  const size_t delimiter_position = receive_buffer_.find(delimiter);
  if (delimiter_position == std::string::npos) {
    if (receive_buffer_.size() > max_line_size) {
      receive_buffer_.clear();
      discarding_line_ = true;
      result = SERIAL_READ_LINE_OVERFLOW;
      return true;
    }
    return false;
  }

  const size_t line_size = delimiter_position + delimiter.size();
  if (line_size > max_line_size) {
    receive_buffer_.erase(0, line_size);
    result = SERIAL_READ_LINE_OVERFLOW;
    return true;
  }

  output.assign(receive_buffer_, 0, line_size);
  receive_buffer_.erase(0, line_size);
  result = static_cast<int>(line_size);
  return true;
}

int Serial::read_until(
  std::string & output, const std::string & delimiter, int timeout_ms,
  size_t max_line_size)
{
  output.clear();
  if (serial_fd_ < 0 || delimiter.empty() || timeout_ms < 0 ||
    max_line_size == 0)
  {
    return SERIAL_READ_LINE_ERROR;
  }

  int result = SERIAL_READ_LINE_NO_DATA;
  if (extract_line_(output, delimiter, max_line_size, result)) {
    return result;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return SERIAL_READ_LINE_TIMEOUT;
    }

    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const int poll_timeout = std::max(1, static_cast<int>(remaining.count()));
    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int poll_result = ::poll(&descriptor, 1, poll_timeout);
    if (poll_result == 0) {
      return SERIAL_READ_LINE_TIMEOUT;
    }
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return SERIAL_READ_LINE_ERROR;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return SERIAL_READ_LINE_ERROR;
    }

    char buffer[256];
    const ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
      receive_buffer_.append(buffer, static_cast<size_t>(bytes_read));
      if (extract_line_(output, delimiter, max_line_size, result)) {
        return result;
      }
    } else if (
      bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
      errno != EINTR)
    {
      return SERIAL_READ_LINE_ERROR;
    }
  }
}
