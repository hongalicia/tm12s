// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_msgs:srv/SetPositions.idl
// generated code does not contain a copyright notice
#include "tm_msgs/srv/detail/set_positions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
tm_msgs__srv__SetPositions_Request__init(tm_msgs__srv__SetPositions_Request * msg)
{
  if (!msg) {
    return false;
  }
  // motion_type
  // positions
  if (!rosidl_runtime_c__double__Sequence__init(&msg->positions, 0)) {
    tm_msgs__srv__SetPositions_Request__fini(msg);
    return false;
  }
  // velocity
  // acc_time
  // blend_percentage
  // fine_goal
  return true;
}

void
tm_msgs__srv__SetPositions_Request__fini(tm_msgs__srv__SetPositions_Request * msg)
{
  if (!msg) {
    return;
  }
  // motion_type
  // positions
  rosidl_runtime_c__double__Sequence__fini(&msg->positions);
  // velocity
  // acc_time
  // blend_percentage
  // fine_goal
}

bool
tm_msgs__srv__SetPositions_Request__are_equal(const tm_msgs__srv__SetPositions_Request * lhs, const tm_msgs__srv__SetPositions_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motion_type
  if (lhs->motion_type != rhs->motion_type) {
    return false;
  }
  // positions
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // acc_time
  if (lhs->acc_time != rhs->acc_time) {
    return false;
  }
  // blend_percentage
  if (lhs->blend_percentage != rhs->blend_percentage) {
    return false;
  }
  // fine_goal
  if (lhs->fine_goal != rhs->fine_goal) {
    return false;
  }
  return true;
}

bool
tm_msgs__srv__SetPositions_Request__copy(
  const tm_msgs__srv__SetPositions_Request * input,
  tm_msgs__srv__SetPositions_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // motion_type
  output->motion_type = input->motion_type;
  // positions
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  // velocity
  output->velocity = input->velocity;
  // acc_time
  output->acc_time = input->acc_time;
  // blend_percentage
  output->blend_percentage = input->blend_percentage;
  // fine_goal
  output->fine_goal = input->fine_goal;
  return true;
}

tm_msgs__srv__SetPositions_Request *
tm_msgs__srv__SetPositions_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__SetPositions_Request * msg = (tm_msgs__srv__SetPositions_Request *)allocator.allocate(sizeof(tm_msgs__srv__SetPositions_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__srv__SetPositions_Request));
  bool success = tm_msgs__srv__SetPositions_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__srv__SetPositions_Request__destroy(tm_msgs__srv__SetPositions_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__srv__SetPositions_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__srv__SetPositions_Request__Sequence__init(tm_msgs__srv__SetPositions_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__SetPositions_Request * data = NULL;

  if (size) {
    data = (tm_msgs__srv__SetPositions_Request *)allocator.zero_allocate(size, sizeof(tm_msgs__srv__SetPositions_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__srv__SetPositions_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__srv__SetPositions_Request__fini(&data[i - 1]);
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
tm_msgs__srv__SetPositions_Request__Sequence__fini(tm_msgs__srv__SetPositions_Request__Sequence * array)
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
      tm_msgs__srv__SetPositions_Request__fini(&array->data[i]);
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

tm_msgs__srv__SetPositions_Request__Sequence *
tm_msgs__srv__SetPositions_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__SetPositions_Request__Sequence * array = (tm_msgs__srv__SetPositions_Request__Sequence *)allocator.allocate(sizeof(tm_msgs__srv__SetPositions_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__srv__SetPositions_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__srv__SetPositions_Request__Sequence__destroy(tm_msgs__srv__SetPositions_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__srv__SetPositions_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__srv__SetPositions_Request__Sequence__are_equal(const tm_msgs__srv__SetPositions_Request__Sequence * lhs, const tm_msgs__srv__SetPositions_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__srv__SetPositions_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__srv__SetPositions_Request__Sequence__copy(
  const tm_msgs__srv__SetPositions_Request__Sequence * input,
  tm_msgs__srv__SetPositions_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__srv__SetPositions_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__srv__SetPositions_Request * data =
      (tm_msgs__srv__SetPositions_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__srv__SetPositions_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__srv__SetPositions_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__srv__SetPositions_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
tm_msgs__srv__SetPositions_Response__init(tm_msgs__srv__SetPositions_Response * msg)
{
  if (!msg) {
    return false;
  }
  // ok
  return true;
}

void
tm_msgs__srv__SetPositions_Response__fini(tm_msgs__srv__SetPositions_Response * msg)
{
  if (!msg) {
    return;
  }
  // ok
}

bool
tm_msgs__srv__SetPositions_Response__are_equal(const tm_msgs__srv__SetPositions_Response * lhs, const tm_msgs__srv__SetPositions_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ok
  if (lhs->ok != rhs->ok) {
    return false;
  }
  return true;
}

bool
tm_msgs__srv__SetPositions_Response__copy(
  const tm_msgs__srv__SetPositions_Response * input,
  tm_msgs__srv__SetPositions_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // ok
  output->ok = input->ok;
  return true;
}

tm_msgs__srv__SetPositions_Response *
tm_msgs__srv__SetPositions_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__SetPositions_Response * msg = (tm_msgs__srv__SetPositions_Response *)allocator.allocate(sizeof(tm_msgs__srv__SetPositions_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__srv__SetPositions_Response));
  bool success = tm_msgs__srv__SetPositions_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__srv__SetPositions_Response__destroy(tm_msgs__srv__SetPositions_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__srv__SetPositions_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__srv__SetPositions_Response__Sequence__init(tm_msgs__srv__SetPositions_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__SetPositions_Response * data = NULL;

  if (size) {
    data = (tm_msgs__srv__SetPositions_Response *)allocator.zero_allocate(size, sizeof(tm_msgs__srv__SetPositions_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__srv__SetPositions_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__srv__SetPositions_Response__fini(&data[i - 1]);
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
tm_msgs__srv__SetPositions_Response__Sequence__fini(tm_msgs__srv__SetPositions_Response__Sequence * array)
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
      tm_msgs__srv__SetPositions_Response__fini(&array->data[i]);
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

tm_msgs__srv__SetPositions_Response__Sequence *
tm_msgs__srv__SetPositions_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__SetPositions_Response__Sequence * array = (tm_msgs__srv__SetPositions_Response__Sequence *)allocator.allocate(sizeof(tm_msgs__srv__SetPositions_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__srv__SetPositions_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__srv__SetPositions_Response__Sequence__destroy(tm_msgs__srv__SetPositions_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__srv__SetPositions_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__srv__SetPositions_Response__Sequence__are_equal(const tm_msgs__srv__SetPositions_Response__Sequence * lhs, const tm_msgs__srv__SetPositions_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__srv__SetPositions_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__srv__SetPositions_Response__Sequence__copy(
  const tm_msgs__srv__SetPositions_Response__Sequence * input,
  tm_msgs__srv__SetPositions_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__srv__SetPositions_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__srv__SetPositions_Response * data =
      (tm_msgs__srv__SetPositions_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__srv__SetPositions_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__srv__SetPositions_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__srv__SetPositions_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
