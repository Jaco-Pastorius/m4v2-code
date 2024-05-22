// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/MPCStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MPCStatus in the package custom_msgs.
typedef struct custom_msgs__msg__MPCStatus
{
  uint64_t timestamp;
  float x[12];
  float xref[12];
  float xnext[12];
  float u[4];
  float uref[4];
  float varphi;
  float tiltvel;
  int32_t status;
  int32_t trackingdone;
  int32_t grounded;
  float comptime;
} custom_msgs__msg__MPCStatus;

// Struct for a sequence of custom_msgs__msg__MPCStatus.
typedef struct custom_msgs__msg__MPCStatus__Sequence
{
  custom_msgs__msg__MPCStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__MPCStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS__STRUCT_H_
