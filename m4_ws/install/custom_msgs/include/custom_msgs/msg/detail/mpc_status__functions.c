// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/MPCStatus.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/mpc_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_msgs__msg__MPCStatus__init(custom_msgs__msg__MPCStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // x
  // xref
  // xnext
  // u
  // uref
  // varphi
  // tiltvel
  // status
  // trackingdone
  // grounded
  // comptime
  return true;
}

void
custom_msgs__msg__MPCStatus__fini(custom_msgs__msg__MPCStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // x
  // xref
  // xnext
  // u
  // uref
  // varphi
  // tiltvel
  // status
  // trackingdone
  // grounded
  // comptime
}

bool
custom_msgs__msg__MPCStatus__are_equal(const custom_msgs__msg__MPCStatus * lhs, const custom_msgs__msg__MPCStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // x
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->x[i] != rhs->x[i]) {
      return false;
    }
  }
  // xref
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->xref[i] != rhs->xref[i]) {
      return false;
    }
  }
  // xnext
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->xnext[i] != rhs->xnext[i]) {
      return false;
    }
  }
  // u
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->u[i] != rhs->u[i]) {
      return false;
    }
  }
  // uref
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->uref[i] != rhs->uref[i]) {
      return false;
    }
  }
  // varphi
  if (lhs->varphi != rhs->varphi) {
    return false;
  }
  // tiltvel
  if (lhs->tiltvel != rhs->tiltvel) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // trackingdone
  if (lhs->trackingdone != rhs->trackingdone) {
    return false;
  }
  // grounded
  if (lhs->grounded != rhs->grounded) {
    return false;
  }
  // comptime
  if (lhs->comptime != rhs->comptime) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__MPCStatus__copy(
  const custom_msgs__msg__MPCStatus * input,
  custom_msgs__msg__MPCStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // x
  for (size_t i = 0; i < 12; ++i) {
    output->x[i] = input->x[i];
  }
  // xref
  for (size_t i = 0; i < 12; ++i) {
    output->xref[i] = input->xref[i];
  }
  // xnext
  for (size_t i = 0; i < 12; ++i) {
    output->xnext[i] = input->xnext[i];
  }
  // u
  for (size_t i = 0; i < 4; ++i) {
    output->u[i] = input->u[i];
  }
  // uref
  for (size_t i = 0; i < 4; ++i) {
    output->uref[i] = input->uref[i];
  }
  // varphi
  output->varphi = input->varphi;
  // tiltvel
  output->tiltvel = input->tiltvel;
  // status
  output->status = input->status;
  // trackingdone
  output->trackingdone = input->trackingdone;
  // grounded
  output->grounded = input->grounded;
  // comptime
  output->comptime = input->comptime;
  return true;
}

custom_msgs__msg__MPCStatus *
custom_msgs__msg__MPCStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MPCStatus * msg = (custom_msgs__msg__MPCStatus *)allocator.allocate(sizeof(custom_msgs__msg__MPCStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__MPCStatus));
  bool success = custom_msgs__msg__MPCStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__MPCStatus__destroy(custom_msgs__msg__MPCStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__MPCStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__MPCStatus__Sequence__init(custom_msgs__msg__MPCStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MPCStatus * data = NULL;

  if (size) {
    data = (custom_msgs__msg__MPCStatus *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__MPCStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__MPCStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__MPCStatus__fini(&data[i - 1]);
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
custom_msgs__msg__MPCStatus__Sequence__fini(custom_msgs__msg__MPCStatus__Sequence * array)
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
      custom_msgs__msg__MPCStatus__fini(&array->data[i]);
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

custom_msgs__msg__MPCStatus__Sequence *
custom_msgs__msg__MPCStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MPCStatus__Sequence * array = (custom_msgs__msg__MPCStatus__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__MPCStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__MPCStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__MPCStatus__Sequence__destroy(custom_msgs__msg__MPCStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__MPCStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__MPCStatus__Sequence__are_equal(const custom_msgs__msg__MPCStatus__Sequence * lhs, const custom_msgs__msg__MPCStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__MPCStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__MPCStatus__Sequence__copy(
  const custom_msgs__msg__MPCStatus__Sequence * input,
  custom_msgs__msg__MPCStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__MPCStatus);
    custom_msgs__msg__MPCStatus * data =
      (custom_msgs__msg__MPCStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__MPCStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__MPCStatus__fini(&data[i]);
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
    if (!custom_msgs__msg__MPCStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
