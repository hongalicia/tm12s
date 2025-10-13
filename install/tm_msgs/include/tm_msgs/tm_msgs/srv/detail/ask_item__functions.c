// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_msgs:srv/AskItem.idl
// generated code does not contain a copyright notice
#include "tm_msgs/srv/detail/ask_item__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `id`
// Member `item`
#include "rosidl_runtime_c/string_functions.h"

bool
tm_msgs__srv__AskItem_Request__init(tm_msgs__srv__AskItem_Request * msg)
{
  if (!msg) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__init(&msg->id)) {
    tm_msgs__srv__AskItem_Request__fini(msg);
    return false;
  }
  // item
  if (!rosidl_runtime_c__String__init(&msg->item)) {
    tm_msgs__srv__AskItem_Request__fini(msg);
    return false;
  }
  // wait_time
  return true;
}

void
tm_msgs__srv__AskItem_Request__fini(tm_msgs__srv__AskItem_Request * msg)
{
  if (!msg) {
    return;
  }
  // id
  rosidl_runtime_c__String__fini(&msg->id);
  // item
  rosidl_runtime_c__String__fini(&msg->item);
  // wait_time
}

bool
tm_msgs__srv__AskItem_Request__are_equal(const tm_msgs__srv__AskItem_Request * lhs, const tm_msgs__srv__AskItem_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // item
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->item), &(rhs->item)))
  {
    return false;
  }
  // wait_time
  if (lhs->wait_time != rhs->wait_time) {
    return false;
  }
  return true;
}

bool
tm_msgs__srv__AskItem_Request__copy(
  const tm_msgs__srv__AskItem_Request * input,
  tm_msgs__srv__AskItem_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // item
  if (!rosidl_runtime_c__String__copy(
      &(input->item), &(output->item)))
  {
    return false;
  }
  // wait_time
  output->wait_time = input->wait_time;
  return true;
}

tm_msgs__srv__AskItem_Request *
tm_msgs__srv__AskItem_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__AskItem_Request * msg = (tm_msgs__srv__AskItem_Request *)allocator.allocate(sizeof(tm_msgs__srv__AskItem_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__srv__AskItem_Request));
  bool success = tm_msgs__srv__AskItem_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__srv__AskItem_Request__destroy(tm_msgs__srv__AskItem_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__srv__AskItem_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__srv__AskItem_Request__Sequence__init(tm_msgs__srv__AskItem_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__AskItem_Request * data = NULL;

  if (size) {
    data = (tm_msgs__srv__AskItem_Request *)allocator.zero_allocate(size, sizeof(tm_msgs__srv__AskItem_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__srv__AskItem_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__srv__AskItem_Request__fini(&data[i - 1]);
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
tm_msgs__srv__AskItem_Request__Sequence__fini(tm_msgs__srv__AskItem_Request__Sequence * array)
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
      tm_msgs__srv__AskItem_Request__fini(&array->data[i]);
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

tm_msgs__srv__AskItem_Request__Sequence *
tm_msgs__srv__AskItem_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__AskItem_Request__Sequence * array = (tm_msgs__srv__AskItem_Request__Sequence *)allocator.allocate(sizeof(tm_msgs__srv__AskItem_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__srv__AskItem_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__srv__AskItem_Request__Sequence__destroy(tm_msgs__srv__AskItem_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__srv__AskItem_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__srv__AskItem_Request__Sequence__are_equal(const tm_msgs__srv__AskItem_Request__Sequence * lhs, const tm_msgs__srv__AskItem_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__srv__AskItem_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__srv__AskItem_Request__Sequence__copy(
  const tm_msgs__srv__AskItem_Request__Sequence * input,
  tm_msgs__srv__AskItem_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__srv__AskItem_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__srv__AskItem_Request * data =
      (tm_msgs__srv__AskItem_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__srv__AskItem_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__srv__AskItem_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__srv__AskItem_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `id`
// Member `value`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
tm_msgs__srv__AskItem_Response__init(tm_msgs__srv__AskItem_Response * msg)
{
  if (!msg) {
    return false;
  }
  // ok
  // id
  if (!rosidl_runtime_c__String__init(&msg->id)) {
    tm_msgs__srv__AskItem_Response__fini(msg);
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__init(&msg->value)) {
    tm_msgs__srv__AskItem_Response__fini(msg);
    return false;
  }
  return true;
}

void
tm_msgs__srv__AskItem_Response__fini(tm_msgs__srv__AskItem_Response * msg)
{
  if (!msg) {
    return;
  }
  // ok
  // id
  rosidl_runtime_c__String__fini(&msg->id);
  // value
  rosidl_runtime_c__String__fini(&msg->value);
}

bool
tm_msgs__srv__AskItem_Response__are_equal(const tm_msgs__srv__AskItem_Response * lhs, const tm_msgs__srv__AskItem_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ok
  if (lhs->ok != rhs->ok) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->value), &(rhs->value)))
  {
    return false;
  }
  return true;
}

bool
tm_msgs__srv__AskItem_Response__copy(
  const tm_msgs__srv__AskItem_Response * input,
  tm_msgs__srv__AskItem_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // ok
  output->ok = input->ok;
  // id
  if (!rosidl_runtime_c__String__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__copy(
      &(input->value), &(output->value)))
  {
    return false;
  }
  return true;
}

tm_msgs__srv__AskItem_Response *
tm_msgs__srv__AskItem_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__AskItem_Response * msg = (tm_msgs__srv__AskItem_Response *)allocator.allocate(sizeof(tm_msgs__srv__AskItem_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__srv__AskItem_Response));
  bool success = tm_msgs__srv__AskItem_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__srv__AskItem_Response__destroy(tm_msgs__srv__AskItem_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__srv__AskItem_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__srv__AskItem_Response__Sequence__init(tm_msgs__srv__AskItem_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__AskItem_Response * data = NULL;

  if (size) {
    data = (tm_msgs__srv__AskItem_Response *)allocator.zero_allocate(size, sizeof(tm_msgs__srv__AskItem_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__srv__AskItem_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__srv__AskItem_Response__fini(&data[i - 1]);
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
tm_msgs__srv__AskItem_Response__Sequence__fini(tm_msgs__srv__AskItem_Response__Sequence * array)
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
      tm_msgs__srv__AskItem_Response__fini(&array->data[i]);
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

tm_msgs__srv__AskItem_Response__Sequence *
tm_msgs__srv__AskItem_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__srv__AskItem_Response__Sequence * array = (tm_msgs__srv__AskItem_Response__Sequence *)allocator.allocate(sizeof(tm_msgs__srv__AskItem_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__srv__AskItem_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__srv__AskItem_Response__Sequence__destroy(tm_msgs__srv__AskItem_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__srv__AskItem_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__srv__AskItem_Response__Sequence__are_equal(const tm_msgs__srv__AskItem_Response__Sequence * lhs, const tm_msgs__srv__AskItem_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__srv__AskItem_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__srv__AskItem_Response__Sequence__copy(
  const tm_msgs__srv__AskItem_Response__Sequence * input,
  tm_msgs__srv__AskItem_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__srv__AskItem_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__srv__AskItem_Response * data =
      (tm_msgs__srv__AskItem_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__srv__AskItem_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__srv__AskItem_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__srv__AskItem_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
