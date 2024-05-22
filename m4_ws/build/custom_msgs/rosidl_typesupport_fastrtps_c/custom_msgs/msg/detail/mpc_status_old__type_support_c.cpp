// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from custom_msgs:msg/MPCStatusOld.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/mpc_status_old__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "custom_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "custom_msgs/msg/detail/mpc_status_old__struct.h"
#include "custom_msgs/msg/detail/mpc_status_old__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _MPCStatusOld__ros_msg_type = custom_msgs__msg__MPCStatusOld;

static bool _MPCStatusOld__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MPCStatusOld__ros_msg_type * ros_message = static_cast<const _MPCStatusOld__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: status
  {
    cdr << ros_message->status;
  }

  // Field name: comptime
  {
    cdr << ros_message->comptime;
  }

  // Field name: input
  {
    size_t size = 4;
    auto array_ptr = ros_message->input;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: xref
  {
    size_t size = 12;
    auto array_ptr = ros_message->xref;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: uref
  {
    size_t size = 4;
    auto array_ptr = ros_message->uref;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: x
  {
    cdr << ros_message->x;
  }

  // Field name: y
  {
    cdr << ros_message->y;
  }

  // Field name: z
  {
    cdr << ros_message->z;
  }

  // Field name: thetaz
  {
    cdr << ros_message->thetaz;
  }

  // Field name: thetay
  {
    cdr << ros_message->thetay;
  }

  // Field name: thetax
  {
    cdr << ros_message->thetax;
  }

  // Field name: dx
  {
    cdr << ros_message->dx;
  }

  // Field name: dy
  {
    cdr << ros_message->dy;
  }

  // Field name: dz
  {
    cdr << ros_message->dz;
  }

  // Field name: omegax
  {
    cdr << ros_message->omegax;
  }

  // Field name: omegay
  {
    cdr << ros_message->omegay;
  }

  // Field name: omegaz
  {
    cdr << ros_message->omegaz;
  }

  // Field name: varphi
  {
    cdr << ros_message->varphi;
  }

  // Field name: tiltvel
  {
    cdr << ros_message->tiltvel;
  }

  return true;
}

static bool _MPCStatusOld__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MPCStatusOld__ros_msg_type * ros_message = static_cast<_MPCStatusOld__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: status
  {
    cdr >> ros_message->status;
  }

  // Field name: comptime
  {
    cdr >> ros_message->comptime;
  }

  // Field name: input
  {
    size_t size = 4;
    auto array_ptr = ros_message->input;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: xref
  {
    size_t size = 12;
    auto array_ptr = ros_message->xref;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: uref
  {
    size_t size = 4;
    auto array_ptr = ros_message->uref;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: x
  {
    cdr >> ros_message->x;
  }

  // Field name: y
  {
    cdr >> ros_message->y;
  }

  // Field name: z
  {
    cdr >> ros_message->z;
  }

  // Field name: thetaz
  {
    cdr >> ros_message->thetaz;
  }

  // Field name: thetay
  {
    cdr >> ros_message->thetay;
  }

  // Field name: thetax
  {
    cdr >> ros_message->thetax;
  }

  // Field name: dx
  {
    cdr >> ros_message->dx;
  }

  // Field name: dy
  {
    cdr >> ros_message->dy;
  }

  // Field name: dz
  {
    cdr >> ros_message->dz;
  }

  // Field name: omegax
  {
    cdr >> ros_message->omegax;
  }

  // Field name: omegay
  {
    cdr >> ros_message->omegay;
  }

  // Field name: omegaz
  {
    cdr >> ros_message->omegaz;
  }

  // Field name: varphi
  {
    cdr >> ros_message->varphi;
  }

  // Field name: tiltvel
  {
    cdr >> ros_message->tiltvel;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_msgs
size_t get_serialized_size_custom_msgs__msg__MPCStatusOld(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MPCStatusOld__ros_msg_type * ros_message = static_cast<const _MPCStatusOld__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name status
  {
    size_t item_size = sizeof(ros_message->status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name comptime
  {
    size_t item_size = sizeof(ros_message->comptime);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name input
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->input;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name xref
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->xref;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name uref
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->uref;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name x
  {
    size_t item_size = sizeof(ros_message->x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y
  {
    size_t item_size = sizeof(ros_message->y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z
  {
    size_t item_size = sizeof(ros_message->z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thetaz
  {
    size_t item_size = sizeof(ros_message->thetaz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thetay
  {
    size_t item_size = sizeof(ros_message->thetay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thetax
  {
    size_t item_size = sizeof(ros_message->thetax);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dx
  {
    size_t item_size = sizeof(ros_message->dx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dy
  {
    size_t item_size = sizeof(ros_message->dy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dz
  {
    size_t item_size = sizeof(ros_message->dz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name omegax
  {
    size_t item_size = sizeof(ros_message->omegax);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name omegay
  {
    size_t item_size = sizeof(ros_message->omegay);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name omegaz
  {
    size_t item_size = sizeof(ros_message->omegaz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name varphi
  {
    size_t item_size = sizeof(ros_message->varphi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tiltvel
  {
    size_t item_size = sizeof(ros_message->tiltvel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MPCStatusOld__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_custom_msgs__msg__MPCStatusOld(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_msgs
size_t max_serialized_size_custom_msgs__msg__MPCStatusOld(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: comptime
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: input
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: xref
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: uref
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: z
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thetaz
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thetay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thetax
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: dx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: dy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: dz
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: omegax
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: omegay
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: omegaz
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: varphi
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tiltvel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _MPCStatusOld__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_custom_msgs__msg__MPCStatusOld(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MPCStatusOld = {
  "custom_msgs::msg",
  "MPCStatusOld",
  _MPCStatusOld__cdr_serialize,
  _MPCStatusOld__cdr_deserialize,
  _MPCStatusOld__get_serialized_size,
  _MPCStatusOld__max_serialized_size
};

static rosidl_message_type_support_t _MPCStatusOld__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MPCStatusOld,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_msgs, msg, MPCStatusOld)() {
  return &_MPCStatusOld__type_support;
}

#if defined(__cplusplus)
}
#endif
