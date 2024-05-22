// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/MPCStatusOld.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MPCStatusOld in the package custom_msgs.
typedef struct custom_msgs__msg__MPCStatusOld
{
  uint64_t timestamp;
  int32_t status;
  float comptime;
  float input[4];
  float xref[12];
  float uref[4];
  float x;
  float y;
  float z;
  float thetaz;
  float thetay;
  float thetax;
  float dx;
  float dy;
  float dz;
  float omegax;
  float omegay;
  float omegaz;
  float varphi;
  float tiltvel;
} custom_msgs__msg__MPCStatusOld;

// Struct for a sequence of custom_msgs__msg__MPCStatusOld.
typedef struct custom_msgs__msg__MPCStatusOld__Sequence
{
  custom_msgs__msg__MPCStatusOld * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__MPCStatusOld__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__MPC_STATUS_OLD__STRUCT_H_
