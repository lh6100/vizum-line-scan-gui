#include "welding_execution/execution_guard.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>

namespace welding_execution
{
namespace
{

void append_u64(std::string * output, std::uint64_t value)
{
  for (unsigned int index = 0; index < 8; ++index) {
    output->push_back(static_cast<char>((value >> (index * 8U)) & 0xffU));
  }
}

void append_string(std::string * output, const std::string & value)
{
  append_u64(output, value.size());
  output->append(value);
}

void append_double(std::string * output, double value)
{
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double representation");
  std::memcpy(&bits, &value, sizeof(value));
  append_u64(output, bits);
}

void append_vector(std::string * output, const std::vector<double> & values)
{
  append_u64(output, values.size());
  for (const double value : values) {append_double(output, value);}
}

}  // namespace

std::string trajectory_sha256(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  std::string canonical;
  append_string(&canonical, trajectory.header.frame_id);
  append_u64(&canonical, static_cast<std::uint64_t>(trajectory.header.stamp.sec));
  append_u64(&canonical, trajectory.header.stamp.nanosec);
  append_u64(&canonical, trajectory.joint_names.size());
  for (const auto & name : trajectory.joint_names) {append_string(&canonical, name);}
  append_u64(&canonical, trajectory.points.size());
  for (const auto & point : trajectory.points) {
    append_vector(&canonical, point.positions);
    append_vector(&canonical, point.velocities);
    append_vector(&canonical, point.accelerations);
    append_vector(&canonical, point.effort);
    append_u64(&canonical, static_cast<std::uint64_t>(point.time_from_start.sec));
    append_u64(&canonical, point.time_from_start.nanosec);
  }
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), canonical.data(), canonical.size()) != 1)
  {return {};}
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &size) != 1) {return {};}
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

bool valid_identifier(const std::string & value)
{
  return !value.empty() && value.size() <= 128U &&
    std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return std::isalnum(character) != 0 || character == '_' || character == '-' || character == '.';
    });
}

bool MotionLease::acquire(const std::string & owner, std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_identifier(owner)) {
    if (error != nullptr) {*error = "motion lease owner is invalid";}
    return false;
  }
  if (owner_) {
    if (error != nullptr) {*error = "motion lease already held by " + *owner_;}
    return false;
  }
  owner_ = owner;
  return true;
}

bool MotionLease::release(const std::string & owner, std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!owner_ || *owner_ != owner) {
    if (error != nullptr) {*error = "motion lease owner mismatch";}
    return false;
  }
  owner_.reset();
  return true;
}

std::optional<std::string> MotionLease::owner() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return owner_;
}

}  // namespace welding_execution
