// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/MPCStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__BUILDER_HPP_

#include "custom_msgs/msg/detail/mpc_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_MPCStatus_comptime
{
public:
  explicit Init_MPCStatus_comptime(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::MPCStatus comptime(::custom_msgs::msg::MPCStatus::_comptime_type arg)
  {
    msg_.comptime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_grounded
{
public:
  explicit Init_MPCStatus_grounded(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_comptime grounded(::custom_msgs::msg::MPCStatus::_grounded_type arg)
  {
    msg_.grounded = std::move(arg);
    return Init_MPCStatus_comptime(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_trackingdone
{
public:
  explicit Init_MPCStatus_trackingdone(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_grounded trackingdone(::custom_msgs::msg::MPCStatus::_trackingdone_type arg)
  {
    msg_.trackingdone = std::move(arg);
    return Init_MPCStatus_grounded(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_status
{
public:
  explicit Init_MPCStatus_status(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_trackingdone status(::custom_msgs::msg::MPCStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MPCStatus_trackingdone(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_tiltvel
{
public:
  explicit Init_MPCStatus_tiltvel(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_status tiltvel(::custom_msgs::msg::MPCStatus::_tiltvel_type arg)
  {
    msg_.tiltvel = std::move(arg);
    return Init_MPCStatus_status(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_varphi
{
public:
  explicit Init_MPCStatus_varphi(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_tiltvel varphi(::custom_msgs::msg::MPCStatus::_varphi_type arg)
  {
    msg_.varphi = std::move(arg);
    return Init_MPCStatus_tiltvel(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_uref
{
public:
  explicit Init_MPCStatus_uref(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_varphi uref(::custom_msgs::msg::MPCStatus::_uref_type arg)
  {
    msg_.uref = std::move(arg);
    return Init_MPCStatus_varphi(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_u
{
public:
  explicit Init_MPCStatus_u(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_uref u(::custom_msgs::msg::MPCStatus::_u_type arg)
  {
    msg_.u = std::move(arg);
    return Init_MPCStatus_uref(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_xnext
{
public:
  explicit Init_MPCStatus_xnext(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_u xnext(::custom_msgs::msg::MPCStatus::_xnext_type arg)
  {
    msg_.xnext = std::move(arg);
    return Init_MPCStatus_u(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_xref
{
public:
  explicit Init_MPCStatus_xref(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_xnext xref(::custom_msgs::msg::MPCStatus::_xref_type arg)
  {
    msg_.xref = std::move(arg);
    return Init_MPCStatus_xnext(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_x
{
public:
  explicit Init_MPCStatus_x(::custom_msgs::msg::MPCStatus & msg)
  : msg_(msg)
  {}
  Init_MPCStatus_xref x(::custom_msgs::msg::MPCStatus::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MPCStatus_xref(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

class Init_MPCStatus_timestamp
{
public:
  Init_MPCStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MPCStatus_x timestamp(::custom_msgs::msg::MPCStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MPCStatus_x(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::MPCStatus>()
{
  return custom_msgs::msg::builder::Init_MPCStatus_timestamp();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__BUILDER_HPP_
