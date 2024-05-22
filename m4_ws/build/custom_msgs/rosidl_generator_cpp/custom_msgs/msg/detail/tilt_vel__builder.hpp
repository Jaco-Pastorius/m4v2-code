// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/TiltVel.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TILT_VEL__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TILT_VEL__BUILDER_HPP_

#include "custom_msgs/msg/detail/tilt_vel__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_TiltVel_value
{
public:
  explicit Init_TiltVel_value(::custom_msgs::msg::TiltVel & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::TiltVel value(::custom_msgs::msg::TiltVel::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::TiltVel msg_;
};

class Init_TiltVel_timestamp
{
public:
  Init_TiltVel_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TiltVel_value timestamp(::custom_msgs::msg::TiltVel::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_TiltVel_value(msg_);
  }

private:
  ::custom_msgs::msg::TiltVel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::TiltVel>()
{
  return custom_msgs::msg::builder::Init_TiltVel_timestamp();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__TILT_VEL__BUILDER_HPP_
