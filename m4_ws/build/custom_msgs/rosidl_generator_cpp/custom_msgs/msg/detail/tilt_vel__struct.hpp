// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/TiltVel.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TILT_VEL__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TILT_VEL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__TiltVel __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__TiltVel __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TiltVel_
{
  using Type = TiltVel_<ContainerAllocator>;

  explicit TiltVel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->value = 0.0f;
    }
  }

  explicit TiltVel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->value = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _value_type =
    float;
  _value_type value;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__value(
    const float & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::TiltVel_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::TiltVel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::TiltVel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::TiltVel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__TiltVel
    std::shared_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__TiltVel
    std::shared_ptr<custom_msgs::msg::TiltVel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TiltVel_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const TiltVel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TiltVel_

// alias to use template instance with default allocator
using TiltVel =
  custom_msgs::msg::TiltVel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__TILT_VEL__STRUCT_HPP_
