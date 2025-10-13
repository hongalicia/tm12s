// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_msgs:msg/FeedbackState.idl
// generated code does not contain a copyright notice
#include "tm_msgs/msg/detail/feedback_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_pos`
// Member `joint_vel`
// Member `joint_tor`
// Member `tool0_pose`
// Member `tool_pose`
// Member `tcp_speed`
// Member `tcp_force`
// Member `joint_tor_average`
// Member `joint_tor_min`
// Member `joint_tor_max`
// Member `cb_digital_output`
// Member `cb_digital_input`
// Member `cb_analog_output`
// Member `cb_analog_input`
// Member `ee_digital_output`
// Member `ee_digital_input`
// Member `ee_analog_output`
// Member `ee_analog_input`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `error_content`
#include "rosidl_runtime_c/string_functions.h"

bool
tm_msgs__msg__FeedbackState__init(tm_msgs__msg__FeedbackState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // is_svr_connected
  // is_sct_connected
  // tmsrv_cperr
  // tmscript_cperr
  // tmsrv_dataerr
  // tmscript_dataerr
  // max_not_connect_in_s
  // disconnection_times
  // joint_pos
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_pos, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // joint_vel
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_vel, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // joint_tor
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_tor, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // tool0_pose
  if (!rosidl_runtime_c__double__Sequence__init(&msg->tool0_pose, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // tool_pose
  if (!rosidl_runtime_c__double__Sequence__init(&msg->tool_pose, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // tcp_speed
  if (!rosidl_runtime_c__double__Sequence__init(&msg->tcp_speed, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // tcp_force
  if (!rosidl_runtime_c__double__Sequence__init(&msg->tcp_force, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // joint_tor_average
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_tor_average, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // joint_tor_min
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_tor_min, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // joint_tor_max
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_tor_max, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // robot_link
  // is_data_table_correct
  // robot_error
  // project_run
  // project_pause
  // safetyguard_a
  // e_stop
  // camera_light
  // error_code
  // project_speed
  // ma_mode
  // robot_light
  // cb_digital_output
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->cb_digital_output, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // cb_digital_input
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->cb_digital_input, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // cb_analog_output
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cb_analog_output, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // cb_analog_input
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cb_analog_input, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // ee_digital_output
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->ee_digital_output, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // ee_digital_input
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->ee_digital_input, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // ee_analog_output
  if (!rosidl_runtime_c__float__Sequence__init(&msg->ee_analog_output, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // ee_analog_input
  if (!rosidl_runtime_c__float__Sequence__init(&msg->ee_analog_input, 0)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  // error_content
  if (!rosidl_runtime_c__String__init(&msg->error_content)) {
    tm_msgs__msg__FeedbackState__fini(msg);
    return false;
  }
  return true;
}

void
tm_msgs__msg__FeedbackState__fini(tm_msgs__msg__FeedbackState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_svr_connected
  // is_sct_connected
  // tmsrv_cperr
  // tmscript_cperr
  // tmsrv_dataerr
  // tmscript_dataerr
  // max_not_connect_in_s
  // disconnection_times
  // joint_pos
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_pos);
  // joint_vel
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_vel);
  // joint_tor
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_tor);
  // tool0_pose
  rosidl_runtime_c__double__Sequence__fini(&msg->tool0_pose);
  // tool_pose
  rosidl_runtime_c__double__Sequence__fini(&msg->tool_pose);
  // tcp_speed
  rosidl_runtime_c__double__Sequence__fini(&msg->tcp_speed);
  // tcp_force
  rosidl_runtime_c__double__Sequence__fini(&msg->tcp_force);
  // joint_tor_average
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_tor_average);
  // joint_tor_min
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_tor_min);
  // joint_tor_max
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_tor_max);
  // robot_link
  // is_data_table_correct
  // robot_error
  // project_run
  // project_pause
  // safetyguard_a
  // e_stop
  // camera_light
  // error_code
  // project_speed
  // ma_mode
  // robot_light
  // cb_digital_output
  rosidl_runtime_c__uint8__Sequence__fini(&msg->cb_digital_output);
  // cb_digital_input
  rosidl_runtime_c__uint8__Sequence__fini(&msg->cb_digital_input);
  // cb_analog_output
  rosidl_runtime_c__float__Sequence__fini(&msg->cb_analog_output);
  // cb_analog_input
  rosidl_runtime_c__float__Sequence__fini(&msg->cb_analog_input);
  // ee_digital_output
  rosidl_runtime_c__uint8__Sequence__fini(&msg->ee_digital_output);
  // ee_digital_input
  rosidl_runtime_c__uint8__Sequence__fini(&msg->ee_digital_input);
  // ee_analog_output
  rosidl_runtime_c__float__Sequence__fini(&msg->ee_analog_output);
  // ee_analog_input
  rosidl_runtime_c__float__Sequence__fini(&msg->ee_analog_input);
  // error_content
  rosidl_runtime_c__String__fini(&msg->error_content);
}

bool
tm_msgs__msg__FeedbackState__are_equal(const tm_msgs__msg__FeedbackState * lhs, const tm_msgs__msg__FeedbackState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // is_svr_connected
  if (lhs->is_svr_connected != rhs->is_svr_connected) {
    return false;
  }
  // is_sct_connected
  if (lhs->is_sct_connected != rhs->is_sct_connected) {
    return false;
  }
  // tmsrv_cperr
  if (lhs->tmsrv_cperr != rhs->tmsrv_cperr) {
    return false;
  }
  // tmscript_cperr
  if (lhs->tmscript_cperr != rhs->tmscript_cperr) {
    return false;
  }
  // tmsrv_dataerr
  if (lhs->tmsrv_dataerr != rhs->tmsrv_dataerr) {
    return false;
  }
  // tmscript_dataerr
  if (lhs->tmscript_dataerr != rhs->tmscript_dataerr) {
    return false;
  }
  // max_not_connect_in_s
  if (lhs->max_not_connect_in_s != rhs->max_not_connect_in_s) {
    return false;
  }
  // disconnection_times
  if (lhs->disconnection_times != rhs->disconnection_times) {
    return false;
  }
  // joint_pos
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_pos), &(rhs->joint_pos)))
  {
    return false;
  }
  // joint_vel
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_vel), &(rhs->joint_vel)))
  {
    return false;
  }
  // joint_tor
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_tor), &(rhs->joint_tor)))
  {
    return false;
  }
  // tool0_pose
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->tool0_pose), &(rhs->tool0_pose)))
  {
    return false;
  }
  // tool_pose
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->tool_pose), &(rhs->tool_pose)))
  {
    return false;
  }
  // tcp_speed
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->tcp_speed), &(rhs->tcp_speed)))
  {
    return false;
  }
  // tcp_force
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->tcp_force), &(rhs->tcp_force)))
  {
    return false;
  }
  // joint_tor_average
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_tor_average), &(rhs->joint_tor_average)))
  {
    return false;
  }
  // joint_tor_min
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_tor_min), &(rhs->joint_tor_min)))
  {
    return false;
  }
  // joint_tor_max
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_tor_max), &(rhs->joint_tor_max)))
  {
    return false;
  }
  // robot_link
  if (lhs->robot_link != rhs->robot_link) {
    return false;
  }
  // is_data_table_correct
  if (lhs->is_data_table_correct != rhs->is_data_table_correct) {
    return false;
  }
  // robot_error
  if (lhs->robot_error != rhs->robot_error) {
    return false;
  }
  // project_run
  if (lhs->project_run != rhs->project_run) {
    return false;
  }
  // project_pause
  if (lhs->project_pause != rhs->project_pause) {
    return false;
  }
  // safetyguard_a
  if (lhs->safetyguard_a != rhs->safetyguard_a) {
    return false;
  }
  // e_stop
  if (lhs->e_stop != rhs->e_stop) {
    return false;
  }
  // camera_light
  if (lhs->camera_light != rhs->camera_light) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // project_speed
  if (lhs->project_speed != rhs->project_speed) {
    return false;
  }
  // ma_mode
  if (lhs->ma_mode != rhs->ma_mode) {
    return false;
  }
  // robot_light
  if (lhs->robot_light != rhs->robot_light) {
    return false;
  }
  // cb_digital_output
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->cb_digital_output), &(rhs->cb_digital_output)))
  {
    return false;
  }
  // cb_digital_input
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->cb_digital_input), &(rhs->cb_digital_input)))
  {
    return false;
  }
  // cb_analog_output
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->cb_analog_output), &(rhs->cb_analog_output)))
  {
    return false;
  }
  // cb_analog_input
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->cb_analog_input), &(rhs->cb_analog_input)))
  {
    return false;
  }
  // ee_digital_output
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->ee_digital_output), &(rhs->ee_digital_output)))
  {
    return false;
  }
  // ee_digital_input
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->ee_digital_input), &(rhs->ee_digital_input)))
  {
    return false;
  }
  // ee_analog_output
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->ee_analog_output), &(rhs->ee_analog_output)))
  {
    return false;
  }
  // ee_analog_input
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->ee_analog_input), &(rhs->ee_analog_input)))
  {
    return false;
  }
  // error_content
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_content), &(rhs->error_content)))
  {
    return false;
  }
  return true;
}

bool
tm_msgs__msg__FeedbackState__copy(
  const tm_msgs__msg__FeedbackState * input,
  tm_msgs__msg__FeedbackState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // is_svr_connected
  output->is_svr_connected = input->is_svr_connected;
  // is_sct_connected
  output->is_sct_connected = input->is_sct_connected;
  // tmsrv_cperr
  output->tmsrv_cperr = input->tmsrv_cperr;
  // tmscript_cperr
  output->tmscript_cperr = input->tmscript_cperr;
  // tmsrv_dataerr
  output->tmsrv_dataerr = input->tmsrv_dataerr;
  // tmscript_dataerr
  output->tmscript_dataerr = input->tmscript_dataerr;
  // max_not_connect_in_s
  output->max_not_connect_in_s = input->max_not_connect_in_s;
  // disconnection_times
  output->disconnection_times = input->disconnection_times;
  // joint_pos
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_pos), &(output->joint_pos)))
  {
    return false;
  }
  // joint_vel
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_vel), &(output->joint_vel)))
  {
    return false;
  }
  // joint_tor
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_tor), &(output->joint_tor)))
  {
    return false;
  }
  // tool0_pose
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->tool0_pose), &(output->tool0_pose)))
  {
    return false;
  }
  // tool_pose
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->tool_pose), &(output->tool_pose)))
  {
    return false;
  }
  // tcp_speed
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->tcp_speed), &(output->tcp_speed)))
  {
    return false;
  }
  // tcp_force
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->tcp_force), &(output->tcp_force)))
  {
    return false;
  }
  // joint_tor_average
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_tor_average), &(output->joint_tor_average)))
  {
    return false;
  }
  // joint_tor_min
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_tor_min), &(output->joint_tor_min)))
  {
    return false;
  }
  // joint_tor_max
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_tor_max), &(output->joint_tor_max)))
  {
    return false;
  }
  // robot_link
  output->robot_link = input->robot_link;
  // is_data_table_correct
  output->is_data_table_correct = input->is_data_table_correct;
  // robot_error
  output->robot_error = input->robot_error;
  // project_run
  output->project_run = input->project_run;
  // project_pause
  output->project_pause = input->project_pause;
  // safetyguard_a
  output->safetyguard_a = input->safetyguard_a;
  // e_stop
  output->e_stop = input->e_stop;
  // camera_light
  output->camera_light = input->camera_light;
  // error_code
  output->error_code = input->error_code;
  // project_speed
  output->project_speed = input->project_speed;
  // ma_mode
  output->ma_mode = input->ma_mode;
  // robot_light
  output->robot_light = input->robot_light;
  // cb_digital_output
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->cb_digital_output), &(output->cb_digital_output)))
  {
    return false;
  }
  // cb_digital_input
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->cb_digital_input), &(output->cb_digital_input)))
  {
    return false;
  }
  // cb_analog_output
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->cb_analog_output), &(output->cb_analog_output)))
  {
    return false;
  }
  // cb_analog_input
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->cb_analog_input), &(output->cb_analog_input)))
  {
    return false;
  }
  // ee_digital_output
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->ee_digital_output), &(output->ee_digital_output)))
  {
    return false;
  }
  // ee_digital_input
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->ee_digital_input), &(output->ee_digital_input)))
  {
    return false;
  }
  // ee_analog_output
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->ee_analog_output), &(output->ee_analog_output)))
  {
    return false;
  }
  // ee_analog_input
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->ee_analog_input), &(output->ee_analog_input)))
  {
    return false;
  }
  // error_content
  if (!rosidl_runtime_c__String__copy(
      &(input->error_content), &(output->error_content)))
  {
    return false;
  }
  return true;
}

tm_msgs__msg__FeedbackState *
tm_msgs__msg__FeedbackState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__FeedbackState * msg = (tm_msgs__msg__FeedbackState *)allocator.allocate(sizeof(tm_msgs__msg__FeedbackState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_msgs__msg__FeedbackState));
  bool success = tm_msgs__msg__FeedbackState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_msgs__msg__FeedbackState__destroy(tm_msgs__msg__FeedbackState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_msgs__msg__FeedbackState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_msgs__msg__FeedbackState__Sequence__init(tm_msgs__msg__FeedbackState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__FeedbackState * data = NULL;

  if (size) {
    data = (tm_msgs__msg__FeedbackState *)allocator.zero_allocate(size, sizeof(tm_msgs__msg__FeedbackState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_msgs__msg__FeedbackState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_msgs__msg__FeedbackState__fini(&data[i - 1]);
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
tm_msgs__msg__FeedbackState__Sequence__fini(tm_msgs__msg__FeedbackState__Sequence * array)
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
      tm_msgs__msg__FeedbackState__fini(&array->data[i]);
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

tm_msgs__msg__FeedbackState__Sequence *
tm_msgs__msg__FeedbackState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_msgs__msg__FeedbackState__Sequence * array = (tm_msgs__msg__FeedbackState__Sequence *)allocator.allocate(sizeof(tm_msgs__msg__FeedbackState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_msgs__msg__FeedbackState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_msgs__msg__FeedbackState__Sequence__destroy(tm_msgs__msg__FeedbackState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_msgs__msg__FeedbackState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_msgs__msg__FeedbackState__Sequence__are_equal(const tm_msgs__msg__FeedbackState__Sequence * lhs, const tm_msgs__msg__FeedbackState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_msgs__msg__FeedbackState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_msgs__msg__FeedbackState__Sequence__copy(
  const tm_msgs__msg__FeedbackState__Sequence * input,
  tm_msgs__msg__FeedbackState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_msgs__msg__FeedbackState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_msgs__msg__FeedbackState * data =
      (tm_msgs__msg__FeedbackState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_msgs__msg__FeedbackState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_msgs__msg__FeedbackState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_msgs__msg__FeedbackState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
