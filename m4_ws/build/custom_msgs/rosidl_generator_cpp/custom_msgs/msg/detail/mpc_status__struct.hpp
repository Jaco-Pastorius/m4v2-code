// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/MPCStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__MPCStatus __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__MPCStatus __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MPCStatus_
{
  using Type = MPCStatus_<ContainerAllocator>;

  explicit MPCStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      std::fill<typename std::array<float, 12>::iterator, float>(this->x.begin(), this->x.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->xref.begin(), this->xref.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->xnext.begin(), this->xnext.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->u.begin(), this->u.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->uref.begin(), this->uref.end(), 0.0f);
      this->varphi = 0.0f;
      this->tiltvel = 0.0f;
      this->status = 0l;
      this->trackingdone = 0l;
      this->grounded = 0l;
      this->comptime = 0.0f;
    }
  }

  explicit MPCStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : x(_alloc),
    xref(_alloc),
    xnext(_alloc),
    u(_alloc),
    uref(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      std::fill<typename std::array<float, 12>::iterator, float>(this->x.begin(), this->x.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->xref.begin(), this->xref.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->xnext.begin(), this->xnext.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->u.begin(), this->u.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->uref.begin(), this->uref.end(), 0.0f);
      this->varphi = 0.0f;
      this->tiltvel = 0.0f;
      this->status = 0l;
      this->trackingdone = 0l;
      this->grounded = 0l;
      this->comptime = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _x_type =
    std::array<float, 12>;
  _x_type x;
  using _xref_type =
    std::array<float, 12>;
  _xref_type xref;
  using _xnext_type =
    std::array<float, 12>;
  _xnext_type xnext;
  using _u_type =
    std::array<float, 4>;
  _u_type u;
  using _uref_type =
    std::array<float, 4>;
  _uref_type uref;
  using _varphi_type =
    float;
  _varphi_type varphi;
  using _tiltvel_type =
    float;
  _tiltvel_type tiltvel;
  using _status_type =
    int32_t;
  _status_type status;
  using _trackingdone_type =
    int32_t;
  _trackingdone_type trackingdone;
  using _grounded_type =
    int32_t;
  _grounded_type grounded;
  using _comptime_type =
    float;
  _comptime_type comptime;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__x(
    const std::array<float, 12> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__xref(
    const std::array<float, 12> & _arg)
  {
    this->xref = _arg;
    return *this;
  }
  Type & set__xnext(
    const std::array<float, 12> & _arg)
  {
    this->xnext = _arg;
    return *this;
  }
  Type & set__u(
    const std::array<float, 4> & _arg)
  {
    this->u = _arg;
    return *this;
  }
  Type & set__uref(
    const std::array<float, 4> & _arg)
  {
    this->uref = _arg;
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
  Type & set__status(
    const int32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__trackingdone(
    const int32_t & _arg)
  {
    this->trackingdone = _arg;
    return *this;
  }
  Type & set__grounded(
    const int32_t & _arg)
  {
    this->grounded = _arg;
    return *this;
  }
  Type & set__comptime(
    const float & _arg)
  {
    this->comptime = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::MPCStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::MPCStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::MPCStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::MPCStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__MPCStatus
    std::shared_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__MPCStatus
    std::shared_ptr<custom_msgs::msg::MPCStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MPCStatus_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->xref != other.xref) {
      return false;
    }
    if (this->xnext != other.xnext) {
      return false;
    }
    if (this->u != other.u) {
      return false;
    }
    if (this->uref != other.uref) {
      return false;
    }
    if (this->varphi != other.varphi) {
      return false;
    }
    if (this->tiltvel != other.tiltvel) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->trackingdone != other.trackingdone) {
      return false;
    }
    if (this->grounded != other.grounded) {
      return false;
    }
    if (this->comptime != other.comptime) {
      return false;
    }
    return true;
  }
  bool operator!=(const MPCStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MPCStatus_

// alias to use template instance with default allocator
using MPCStatus =
  custom_msgs::msg::MPCStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__STRUCT_HPP_
