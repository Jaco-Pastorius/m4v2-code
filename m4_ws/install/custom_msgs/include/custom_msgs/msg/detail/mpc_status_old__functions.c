// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/MPCStatusOld.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/mpc_status_old__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_msgs__msg__MPCStatusOld__init(custom_msgs__msg__MPCStatusOld * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // status
  // comptime
  // input
  // xref
  // uref
  // x
  // y
  // z
  // thetaz
  // thetay
  // thetax
  // dx
  // dy
  // dz
  // omegax
  // omegay
  // omegaz
  // varphi
  // tiltvel
  return true;
}

void
custom_msgs__msg__MPCStatusOld__fini(custom_msgs__msg__MPCStatusOld * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // status
  // comptime
  // input
  // xref
  // uref
  // x
  // y
  // z
  // thetaz
  // thetay
  // thetax
  // dx
  // dy
  // dz
  // omegax
  // omegay
  // omegaz
  // varphi
  // tiltvel
}

bool
custom_msgs__msg__MPCStatusOld__are_equal(const custom_msgs__msg__MPCStatusOld * lhs, const custom_msgs__msg__MPCStatusOld * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // comptime
  if (lhs->comptime != rhs->comptime) {
    return false;
  }
  // input
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->input[i] != rhs->input[i]) {
      return false;
    }
  }
  // xref
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->xref[i] != rhs->xref[i]) {
      return false;
    }
  }
  // uref
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->uref[i] != rhs->uref[i]) {
      return false;
    }
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // thetaz
  if (lhs->thetaz != rhs->thetaz) {
    return false;
  }
  // thetay
  if (lhs->thetay != rhs->thetay) {
    return false;
  }
  // thetax
  if (lhs->thetax != rhs->thetax) {
    return false;
  }
  // dx
  if (lhs->dx != rhs->dx) {
    return false;
  }
  // dy
  if (lhs->dy != rhs->dy) {
    return false;
  }
  // dz
  if (lhs->dz != rhs->dz) {
    return false;
  }
  // omegax
  if (lhs->omegax != rhs->omegax) {
    return false;
  }
  // omegay
  if (lhs->omegay != rhs->omegay) {
    return false;
  }
  // omegaz
  if (lhs->omegaz != rhs->omegaz) {
    return false;
  }
  // varphi
  if (lhs->varphi != rhs->varphi) {
    return false;
  }
  // tiltvel
  if (lhs->tiltvel != rhs->tiltvel) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__MPCStatusOld__copy(
  const custom_msgs__msg__MPCStatusOld * input,
  custom_msgs__msg__MPCStatusOld * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // status
  output->status = input->status;
  // comptime
  output->comptime = input->comptime;
  // input
  for (size_t i = 0; i < 4; ++i) {
    output->input[i] = input->input[i];
  }
  // xref
  for (size_t i = 0; i < 12; ++i) {
    output->xref[i] = input->xref[i];
  }
  // uref
  for (size_t i = 0; i < 4; ++i) {
    output->uref[i] = input->uref[i];
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // thetaz
  output->thetaz = input->thetaz;
  // thetay
  output->thetay = input->thetay;
  // thetax
  output->thetax = input->thetax;
  // dx
  output->dx = input->dx;
  // dy
  output->dy = input->dy;
  // dz
  output->dz = input->dz;
  // omegax
  output->omegax = input->omegax;
  // omegay
  output->omegay = input->omegay;
  // omegaz
  output->omegaz = input->omegaz;
  // varphi
  output->varphi = input->varphi;
  // tiltvel
  output->tiltvel = input->tiltvel;
  return true;
}

custom_msgs__msg__MPCStatusOld *
custom_msgs__msg__MPCStatusOld__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MPCStatusOld * msg = (custom_msgs__msg__MPCStatusOld *)allocator.allocate(sizeof(custom_msgs__msg__MPCStatusOld), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__MPCStatusOld));
  bool success = custom_msgs__msg__MPCStatusOld__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__MPCStatusOld__destroy(custom_msgs__msg__MPCStatusOld * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__MPCStatusOld__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__MPCStatusOld__Sequence__init(custom_msgs__msg__MPCStatusOld__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MPCStatusOld * data = NULL;

  if (size) {
    data = (custom_msgs__msg__MPCStatusOld *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__MPCStatusOld), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__MPCStatusOld__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__MPCStatusOld__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_msgs__msg__MPCStatusOld__Sequence__fini(custom_msgs__msg__MPCStatusOld__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__MPCStatusOld__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_msgs__msg__MPCStatusOld__Sequence *
custom_msgs__msg__MPCStatusOld__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MPCStatusOld__Sequence * array = (custom_msgs__msg__MPCStatusOld__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__MPCStatusOld__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__MPCStatusOld__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__MPCStatusOld__Sequence__destroy(custom_msgs__msg__MPCStatusOld__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__MPCStatusOld__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__MPCStatusOld__Sequence__are_equal(const custom_msgs__msg__MPCStatusOld__Sequence * lhs, const custom_msgs__msg__MPCStatusOld__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__MPCStatusOld__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__MPCStatusOld__Sequence__copy(
  const custom_msgs__msg__MPCStatusOld__Sequence * input,
  custom_msgs__msg__MPCStatusOld__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__MPCStatusOld);
    custom_msgs__msg__MPCStatusOld * data =
      (custom_msgs__msg__MPCStatusOld *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__MPCStatusOld__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__MPCStatusOld__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__MPCStatusOld__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
