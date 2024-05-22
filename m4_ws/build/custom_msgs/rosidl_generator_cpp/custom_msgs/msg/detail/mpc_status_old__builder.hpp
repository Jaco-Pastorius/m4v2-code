// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/MPCStatusOld.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__BUILDER_HPP_

#include "custom_msgs/msg/detail/mpc_status_old__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_MPCStatusOld_tiltvel
{
public:
  explicit Init_MPCStatusOld_tiltvel(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::MPCStatusOld tiltvel(::custom_msgs::msg::MPCStatusOld::_tiltvel_type arg)
  {
    msg_.tiltvel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_varphi
{
public:
  explicit Init_MPCStatusOld_varphi(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_tiltvel varphi(::custom_msgs::msg::MPCStatusOld::_varphi_type arg)
  {
    msg_.varphi = std::move(arg);
    return Init_MPCStatusOld_tiltvel(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_omegaz
{
public:
  explicit Init_MPCStatusOld_omegaz(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_varphi omegaz(::custom_msgs::msg::MPCStatusOld::_omegaz_type arg)
  {
    msg_.omegaz = std::move(arg);
    return Init_MPCStatusOld_varphi(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_omegay
{
public:
  explicit Init_MPCStatusOld_omegay(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_omegaz omegay(::custom_msgs::msg::MPCStatusOld::_omegay_type arg)
  {
    msg_.omegay = std::move(arg);
    return Init_MPCStatusOld_omegaz(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_omegax
{
public:
  explicit Init_MPCStatusOld_omegax(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_omegay omegax(::custom_msgs::msg::MPCStatusOld::_omegax_type arg)
  {
    msg_.omegax = std::move(arg);
    return Init_MPCStatusOld_omegay(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_dz
{
public:
  explicit Init_MPCStatusOld_dz(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_omegax dz(::custom_msgs::msg::MPCStatusOld::_dz_type arg)
  {
    msg_.dz = std::move(arg);
    return Init_MPCStatusOld_omegax(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_dy
{
public:
  explicit Init_MPCStatusOld_dy(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_dz dy(::custom_msgs::msg::MPCStatusOld::_dy_type arg)
  {
    msg_.dy = std::move(arg);
    return Init_MPCStatusOld_dz(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_dx
{
public:
  explicit Init_MPCStatusOld_dx(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_dy dx(::custom_msgs::msg::MPCStatusOld::_dx_type arg)
  {
    msg_.dx = std::move(arg);
    return Init_MPCStatusOld_dy(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_thetax
{
public:
  explicit Init_MPCStatusOld_thetax(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_dx thetax(::custom_msgs::msg::MPCStatusOld::_thetax_type arg)
  {
    msg_.thetax = std::move(arg);
    return Init_MPCStatusOld_dx(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_thetay
{
public:
  explicit Init_MPCStatusOld_thetay(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_thetax thetay(::custom_msgs::msg::MPCStatusOld::_thetay_type arg)
  {
    msg_.thetay = std::move(arg);
    return Init_MPCStatusOld_thetax(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_thetaz
{
public:
  explicit Init_MPCStatusOld_thetaz(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_thetay thetaz(::custom_msgs::msg::MPCStatusOld::_thetaz_type arg)
  {
    msg_.thetaz = std::move(arg);
    return Init_MPCStatusOld_thetay(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_z
{
public:
  explicit Init_MPCStatusOld_z(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_thetaz z(::custom_msgs::msg::MPCStatusOld::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MPCStatusOld_thetaz(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_y
{
public:
  explicit Init_MPCStatusOld_y(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_z y(::custom_msgs::msg::MPCStatusOld::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MPCStatusOld_z(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_x
{
public:
  explicit Init_MPCStatusOld_x(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_y x(::custom_msgs::msg::MPCStatusOld::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MPCStatusOld_y(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_uref
{
public:
  explicit Init_MPCStatusOld_uref(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_x uref(::custom_msgs::msg::MPCStatusOld::_uref_type arg)
  {
    msg_.uref = std::move(arg);
    return Init_MPCStatusOld_x(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_xref
{
public:
  explicit Init_MPCStatusOld_xref(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_uref xref(::custom_msgs::msg::MPCStatusOld::_xref_type arg)
  {
    msg_.xref = std::move(arg);
    return Init_MPCStatusOld_uref(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_input
{
public:
  explicit Init_MPCStatusOld_input(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_xref input(::custom_msgs::msg::MPCStatusOld::_input_type arg)
  {
    msg_.input = std::move(arg);
    return Init_MPCStatusOld_xref(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_comptime
{
public:
  explicit Init_MPCStatusOld_comptime(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_input comptime(::custom_msgs::msg::MPCStatusOld::_comptime_type arg)
  {
    msg_.comptime = std::move(arg);
    return Init_MPCStatusOld_input(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_status
{
public:
  explicit Init_MPCStatusOld_status(::custom_msgs::msg::MPCStatusOld & msg)
  : msg_(msg)
  {}
  Init_MPCStatusOld_comptime status(::custom_msgs::msg::MPCStatusOld::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MPCStatusOld_comptime(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

class Init_MPCStatusOld_timestamp
{
public:
  Init_MPCStatusOld_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MPCStatusOld_status timestamp(::custom_msgs::msg::MPCStatusOld::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MPCStatusOld_status(msg_);
  }

private:
  ::custom_msgs::msg::MPCStatusOld msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::MPCStatusOld>()
{
  return custom_msgs::msg::builder::Init_MPCStatusOld_timestamp();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__BUILDER_HPP_
