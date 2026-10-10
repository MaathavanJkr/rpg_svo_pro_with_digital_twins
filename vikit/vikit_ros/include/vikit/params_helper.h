/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <algorithm>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

namespace detail {

// ROS 1 style nested names ("a/b") become ROS 2 parameter names ("a.b").
inline std::string toRosParamName(std::string name)
{
  std::replace(name.begin(), name.end(), '/', '.');
  return name;
}

// ROS 2 parameters are strictly typed, while the SVO yaml files freely mix
// e.g. `1` and `1.0`. Convert between the numeric types like ROS 1 did.
inline bool fromParameterValue(const rclcpp::ParameterValue& v, bool* out)
{
  switch (v.get_type())
  {
    case rclcpp::ParameterType::PARAMETER_BOOL: *out = v.get<bool>(); return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER: *out = v.get<int64_t>() != 0; return true;
    default: return false;
  }
}

inline bool fromParameterValue(const rclcpp::ParameterValue& v, double* out)
{
  switch (v.get_type())
  {
    case rclcpp::ParameterType::PARAMETER_DOUBLE: *out = v.get<double>(); return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER: *out = static_cast<double>(v.get<int64_t>()); return true;
    default: return false;
  }
}

inline bool fromParameterValue(const rclcpp::ParameterValue& v, float* out)
{
  double d;
  if (!fromParameterValue(v, &d))
    return false;
  *out = static_cast<float>(d);
  return true;
}

inline bool fromParameterValue(const rclcpp::ParameterValue& v, int* out)
{
  switch (v.get_type())
  {
    case rclcpp::ParameterType::PARAMETER_INTEGER: *out = static_cast<int>(v.get<int64_t>()); return true;
    case rclcpp::ParameterType::PARAMETER_DOUBLE: *out = static_cast<int>(v.get<double>()); return true;
    case rclcpp::ParameterType::PARAMETER_BOOL: *out = v.get<bool>() ? 1 : 0; return true;
    default: return false;
  }
}

inline bool fromParameterValue(const rclcpp::ParameterValue& v, std::string* out)
{
  if (v.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return false;
  *out = v.get<std::string>();
  return true;
}

} // namespace detail

template<typename T>
bool getParam(const rclcpp::Node::SharedPtr& nh, const std::string& name, T* value)
{
  rclcpp::Parameter p;
  if (!nh->get_parameter(detail::toRosParamName(name), p) ||
      p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    return false;
  }
  if (!detail::fromParameterValue(p.get_parameter_value(), value))
  {
    RCLCPP_ERROR_STREAM(nh->get_logger(), "Parameter " << name << " has unexpected type "
                        << p.get_type_name());
    return false;
  }
  return true;
}

inline bool hasParam(const rclcpp::Node::SharedPtr& nh, const std::string& name)
{
  return nh->has_parameter(detail::toRosParamName(name));
}

template<typename T>
T param(const rclcpp::Node::SharedPtr& nh, const std::string& name, const T& defaultValue,
        const bool silent=false)
{
  T v;
  if (getParam(nh, name, &v))
  {
    if (!silent)
    {
      RCLCPP_INFO_STREAM(nh->get_logger(), "Found parameter: " << name << ", value: " << v);
    }
    return v;
  }
  if (!silent)
  {
    RCLCPP_WARN_STREAM(nh->get_logger(), "Cannot find value for parameter: " << name
                       << ", assigning default: " << defaultValue);
  }
  // Declare the default so it shows up in `ros2 param list/get`.
  const std::string ros_name = detail::toRosParamName(name);
  if (!nh->has_parameter(ros_name))
  {
    try
    {
      nh->declare_parameter(ros_name, rclcpp::ParameterValue(defaultValue));
    }
    catch (const std::exception&)
    {
      // Not fatal: the default is still returned.
    }
  }
  return defaultValue;
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
