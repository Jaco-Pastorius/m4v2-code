// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/MPCStatusOld.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__MPCStatusOld __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__MPCStatusOld __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MPCStatusOld_
{
  using Type = MPCStatusOld_<ContainerAllocator>;

  explicit MPCStatusOld_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->status = 0l;
      this->comptime = 0.0f;
      std::fill<typename std::array<float, 4>::iterator, float>(this->input.begin(), this->input.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->xref.begin(), this->xref.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->uref.begin(), this->uref.end(), 0.0f);
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->thetaz = 0.0f;
      this->thetay = 0.0f;
      this->thetax = 0.0f;
      this->dx = 0.0f;
      this->dy = 0.0f;
      this->dz = 0.0f;
      this->omegax = 0.0f;
      this->omegay = 0.0f;
      this->omegaz = 0.0f;
      this->varphi = 0.0f;
      this->tiltvel = 0.0f;
    }
  }

  explicit MPCStatusOld_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : input(_alloc),
    xref(_alloc),
    uref(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->status = 0l;
      this->comptime = 0.0f;
      std::fill<typename std::array<float, 4>::iterator, float>(this->input.begin(), this->input.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->xref.begin(), this->xref.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->uref.begin(), this->uref.end(), 0.0f);
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->thetaz = 0.0f;
      this->thetay = 0.0f;
      this->thetax = 0.0f;
      this->dx = 0.0f;
      this->dy = 0.0f;
      this->dz = 0.0f;
      this->omegax = 0.0f;
      this->omegay = 0.0f;
      this->omegaz = 0.0f;
      this->varphi = 0.0f;
      this->tiltvel = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _status_type =
    int32_t;
  _status_type status;
  using _comptime_type =
    float;
  _comptime_type comptime;
  using _input_type =
    std::array<float, 4>;
  _input_type input;
  using _xref_type =
    std::array<float, 12>;
  _xref_type xref;
  using _uref_type =
    std::array<float, 4>;
  _uref_type uref;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _thetaz_type =
    float;
  _thetaz_type thetaz;
  using _thetay_type =
    float;
  _thetay_type thetay;
  using _thetax_type =
    float;
  _thetax_type thetax;
  using _dx_type =
    float;
  _dx_type dx;
  using _dy_type =
    float;
  _dy_type dy;
  using _dz_type =
    float;
  _dz_type dz;
  using _omegax_type =
    float;
  _omegax_type omegax;
  using _omegay_type =
    float;
  _omegay_type omegay;
  using _omegaz_type =
    float;
  _omegaz_type omegaz;
  using _varphi_type =
    float;
  _varphi_type varphi;
  using _tiltvel_type =
    float;
  _tiltvel_type tiltvel;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__status(
    const int32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__comptime(
    const float & _arg)
  {
    this->comptime = _arg;
    return *this;
  }
  Type & set__input(
    const std::array<float, 4> & _arg)
  {
    this->input = _arg;
    return *this;
  }
  Type & set__xref(
    const std::array<float, 12> & _arg)
  {
    this->xref = _arg;
    return *this;
  }
  Type & set__uref(
    const std::array<float, 4> & _arg)
  {
    this->uref = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__thetaz(
    const float & _arg)
  {
    this->thetaz = _arg;
    return *this;
  }
  Type & set__thetay(
    const float & _arg)
  {
    this->thetay = _arg;
    return *this;
  }
  Type & set__thetax(
    const float & _arg)
  {
    this->thetax = _arg;
    return *this;
  }
  Type & set__dx(
    const float & _arg)
  {
    this->dx = _arg;
    return *this;
  }
  Type & set__dy(
    const float & _arg)
  {
    this->dy = _arg;
    return *this;
  }
  Type & set__dz(
    const float & _arg)
  {
    this->dz = _arg;
    return *this;
  }
  Type & set__omegax(
    const float & _arg)
  {
    this->omegax = _arg;
    return *this;
  }
  Type & set__omegay(
    const float & _arg)
  {
    this->omegay = _arg;
    return *this;
  }
  Type & set__omegaz(
    const float & _arg)
  {
    this->omegaz = _arg;
    return *this;
  }
  Type & set__varphi(
    const float & _arg)
  {
    this->varphi = _arg;
    return *this;
  }
  Type & set__tiltvel(
    const float & _arg)
  {
    this->tiltvel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::MPCStatusOld_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::MPCStatusOld_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::MPCStatusOld_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::MPCStatusOld_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__MPCStatusOld
    std::shared_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__MPCStatusOld
    std::shared_ptr<custom_msgs::msg::MPCStatusOld_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MPCStatusOld_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->comptime != other.comptime) {
      return false;
    }
    if (this->input != other.input) {
      return false;
    }
    if (this->xref != other.xref) {
      return false;
    }
    if (this->uref != other.uref) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->thetaz != other.thetaz) {
      return false;
    }
    if (this->thetay != other.thetay) {
      return false;
    }
    if (this->thetax != other.thetax) {
      return false;
    }
    if (this->dx != other.dx) {
      return false;
    }
    if (this->dy != other.dy) {
      return false;
    }
    if (this->dz != other.dz) {
      return false;
    }
    if (this->omegax != other.omegax) {
      return false;
    }
    if (this->omegay != other.omegay) {
      return false;
    }
    if (this->omegaz != other.omegaz) {
      return false;
    }
    if (this->varphi != other.varphi) {
      return false;
    }
    if (this->tiltvel != other.tiltvel) {
      return false;
    }
    return true;
  }
  bool operator!=(const MPCStatusOld_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MPCStatusOld_

// alias to use template instance with default allocator
using MPCStatusOld =
  custom_msgs::msg::MPCStatusOld_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__STRUCT_HPP_
