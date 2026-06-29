#include "iahrs_ros2_driver/iahrs_driver.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

namespace
{

constexpr char END_DATA[] = "\r\n";
constexpr auto COMMAND_DELAY = std::chrono::milliseconds(100);

bool parse_double(const std::string & token, double & value)
{
  if (token.empty()) {
    return false;
  }

  char *end = nullptr;
  errno = 0;
  const double parsed = std::strtod(token.c_str(), &end);
  if (end == token.c_str() || errno == ERANGE || !std::isfinite(parsed)) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != '\0') {
    return false;
  }

  value = parsed;
  return true;
}

bool parse_uint64(const std::string & token, uint64_t & value)
{
  if (token.empty() || token.front() == '-') {
    return false;
  }

  char *end = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(token.c_str(), &end, 10);
  if (end == token.c_str() || errno == ERANGE) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != '\0' ||
    parsed > std::numeric_limits<uint64_t>::max())
  {
    return false;
  }

  value = static_cast<uint64_t>(parsed);
  return true;
}

std::vector<std::string> split_fields(std::string line)
{
  while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }
  if (!line.empty() && line.back() == ',') {
    line.pop_back();
  }

  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

IAHRSDriver::IAHRSDriver(const std::string & port)
: serial_(port, 115200) {}

bool IAHRSDriver::write_command_(const std::string & command)
{
  const std::string wire_command = command + '\n';
  return serial_.write(wire_command.data(), wire_command.size()) ==
         static_cast<int>(wire_command.size());
}

bool IAHRSDriver::send_obj_(const std::string & obj)
{
  if (!write_command_(obj)) {
    return false;
  }
  std::this_thread::sleep_for(COMMAND_DELAY);
  serial_.flush();
  return true;
}

bool IAHRSDriver::send_obj_(
  const std::string & obj, const std::string & data)
{
  if (!write_command_(obj + "=" + data)) {
    return false;
  }

  std::string response;
  const int result =
    serial_.read_until(response, END_DATA, 500, MAX_RESPONSE_SIZE);
  if (result <= 0) {
    return false;
  }

  std::this_thread::sleep_for(COMMAND_DELAY);
  serial_.flush();
  return true;
}

bool IAHRSDriver::initialize()
{
  return serial_.open();
}

bool IAHRSDriver::reboot()
{
  serial_.flush();
  if (!send_obj_(IAHRS_OBJ_SET_SENSOR_RESTART)) {
    return false;
  }

  std::string response;
  for (int index = 0; index < 4; ++index) {
    const int result =
      serial_.read_until(response, END_DATA, 500, MAX_RESPONSE_SIZE);
    if (result <= 0) {
      return false;
    }

    if (index == 1 && response != std::string("iAHRS") + END_DATA) {
      return false;
    }

    const std::string software_prefix = "S/W ver: ";
    const std::string hardware_prefix = "H/W ver: ";
    if (index == 2) {
      if (response.rfind(software_prefix, 0) != 0 ||
        std::sscanf(
              response.c_str() + software_prefix.size(), "%d.%d",
              &version_.sw.major, &version_.sw.minor) != 2)
      {
        return false;
      }
    } else if (index == 3) {
      if (response.rfind(hardware_prefix, 0) != 0 ||
        std::sscanf(
              response.c_str() + hardware_prefix.size(), "%d.%d",
              &version_.hw.major, &version_.hw.minor) != 2)
      {
        return false;
      }
    }
  }
  return true;
}

bool IAHRSDriver::set_sync(uint16_t target_mask, uint16_t period_ms)
{
  if (target_mask >= IAHRS_DRIVER_SYNC_FLAG_MAX ||
    period_ms == 0 || period_ms > 60000)
  {
    return false;
  }

  std::ostringstream mask_stream;
  mask_stream << "0x" << std::hex << target_mask;
  if (!send_obj_(IAHRS_OBJ_SETW_SYNC_DATA_MODE, "1") ||
    !send_obj_(
          IAHRS_OBJ_SETW_SYNC_DATA_PERIOD, std::to_string(period_ms)) ||
    !send_obj_(IAHRS_OBJ_SETW_SYNC_DATA_TYPE, mask_stream.str()))
  {
    return false;
  }

  sync_mask_ = target_mask;
  return true;
}

bool IAHRSDriver::set_option(bool state)
{
  return send_obj_(IAHRS_OBJ_SETW_OPTION, state ? "1" : "0");
}

bool IAHRSDriver::reset_euler_angle()
{
  return send_obj_(IAHRS_OBJ_SET_RESET_EULER_ANGLE);
}

bool IAHRSDriver::fetch_sync_data(
  iahrs_driver_sync_data_t & data, int timeout_ms)
{
  std::string line;
  const int result =
    serial_.read_until(line, END_DATA, timeout_ms, MAX_RESPONSE_SIZE);
  if (result <= 0) {
    return false;
  }
  return parse_sync_data(line, sync_mask_, data);
}

bool IAHRSDriver::parse_sync_data(
  const std::string & line, uint16_t sync_mask,
  iahrs_driver_sync_data_t & data)
{
  if (sync_mask == IAHRS_DRIVER_SYNC_FLAG_NONE ||
    sync_mask >= IAHRS_DRIVER_SYNC_FLAG_MAX)
  {
    return false;
  }

  const std::vector<std::string> fields = split_fields(line);
  size_t field_index = 0;
  iahrs_driver_sync_data_t parsed{};

  const auto next_double = [&](double & value) {
      return field_index < fields.size() &&
             parse_double(fields[field_index++], value);
    };
  const auto next_uint64 = [&](uint64_t & value) {
      return field_index < fields.size() &&
             parse_uint64(fields[field_index++], value);
    };
  const auto next_axis = [&](axis_data_t & axis) {
      return next_double(axis.x) &&
             next_double(axis.y) &&
             next_double(axis.z);
    };

  for (uint16_t flag = IAHRS_DRIVER_SYNC_FLAG_1MS_TIME;
    flag < IAHRS_DRIVER_SYNC_FLAG_MAX; flag <<= 1)
  {
    if ((sync_mask & flag) == 0) {
      continue;
    }

    bool valid = false;
    switch (flag) {
      case IAHRS_DRIVER_SYNC_FLAG_1MS_TIME:
        valid = next_uint64(parsed.one_ms_time);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_TEMP:
        valid = next_double(parsed.temperature);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL:
        valid = next_axis(parsed.accel);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO:
        valid = next_axis(parsed.gyro);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG:
        valid = next_axis(parsed.mag);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_GRAVITY_REMOVED_ACCEL:
        valid = next_axis(parsed.accel_gravity_removed);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_EULER_ANGLE:
        valid = next_axis(parsed.euler_angle);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_QUATERNION:
        valid =
          next_double(parsed.quaternion_angle.w) &&
          next_double(parsed.quaternion_angle.x) &&
          next_double(parsed.quaternion_angle.y) &&
          next_double(parsed.quaternion_angle.z);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_GLOBAL_VELOCITY:
        valid = next_axis(parsed.velocity);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_GLOBAL_POSITION:
        valid = next_axis(parsed.position);
        break;
      case IAHRS_DRIVER_SYNC_FLAG_VIBRATION:
        valid = next_axis(parsed.vibration);
        break;
      default:
        return false;
    }
    if (!valid) {
      return false;
    }
  }

  if (field_index != fields.size()) {
    return false;
  }

  data = parsed;
  return true;
}
