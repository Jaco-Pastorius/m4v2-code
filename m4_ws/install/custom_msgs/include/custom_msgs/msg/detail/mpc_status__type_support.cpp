// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from custom_msgs:msg/MPCStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_msgs/msg/detail/mpc_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace custom_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MPCStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) custom_msgs::msg::MPCStatus(_init);
}

void MPCStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<custom_msgs::msg::MPCStatus *>(message_memory);
  typed_message->~MPCStatus();
}

size_t size_function__MPCStatus__x(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__MPCStatus__x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__MPCStatus__x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__MPCStatus__xref(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__MPCStatus__xref(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__MPCStatus__xref(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__MPCStatus__xnext(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__MPCStatus__xnext(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__MPCStatus__xnext(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__MPCStatus__u(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MPCStatus__u(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MPCStatus__u(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

size_t size_function__MPCStatus__uref(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__MPCStatus__uref(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__MPCStatus__uref(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MPCStatus_message_member_array[12] = {
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, x),  // bytes offset in struct
    nullptr,  // default value
    size_function__MPCStatus__x,  // size() function pointer
    get_const_function__MPCStatus__x,  // get_const(index) function pointer
    get_function__MPCStatus__x,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "xref",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, xref),  // bytes offset in struct
    nullptr,  // default value
    size_function__MPCStatus__xref,  // size() function pointer
    get_const_function__MPCStatus__xref,  // get_const(index) function pointer
    get_function__MPCStatus__xref,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "xnext",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, xnext),  // bytes offset in struct
    nullptr,  // default value
    size_function__MPCStatus__xnext,  // size() function pointer
    get_const_function__MPCStatus__xnext,  // get_const(index) function pointer
    get_function__MPCStatus__xnext,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "u",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, u),  // bytes offset in struct
    nullptr,  // default value
    size_function__MPCStatus__u,  // size() function pointer
    get_const_function__MPCStatus__u,  // get_const(index) function pointer
    get_function__MPCStatus__u,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "uref",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, uref),  // bytes offset in struct
    nullptr,  // default value
    size_function__MPCStatus__uref,  // size() function pointer
    get_const_function__MPCStatus__uref,  // get_const(index) function pointer
    get_function__MPCStatus__uref,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "varphi",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, varphi),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tiltvel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, tiltvel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "trackingdone",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, trackingdone),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "grounded",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, grounded),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "comptime",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs::msg::MPCStatus, comptime),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MPCStatus_message_members = {
  "custom_msgs::msg",  // message namespace
  "MPCStatus",  // message name
  12,  // number of fields
  sizeof(custom_msgs::msg::MPCStatus),
  MPCStatus_message_member_array,  // message members
  MPCStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MPCStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MPCStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MPCStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace custom_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_msgs::msg::MPCStatus>()
{
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::MPCStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_msgs, msg, MPCStatus)() {
  return &::custom_msgs::msg::rosidl_typesupport_introspection_cpp::MPCStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
