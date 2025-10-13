// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tm_msgs:msg/FeedbackState.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__MSG__DETAIL__FEEDBACK_STATE__BUILDER_HPP_
#define TM_MSGS__MSG__DETAIL__FEEDBACK_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tm_msgs/msg/detail/feedback_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tm_msgs
{

namespace msg
{

namespace builder
{

class Init_FeedbackState_error_content
{
public:
  explicit Init_FeedbackState_error_content(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  ::tm_msgs::msg::FeedbackState error_content(::tm_msgs::msg::FeedbackState::_error_content_type arg)
  {
    msg_.error_content = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_ee_analog_input
{
public:
  explicit Init_FeedbackState_ee_analog_input(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_error_content ee_analog_input(::tm_msgs::msg::FeedbackState::_ee_analog_input_type arg)
  {
    msg_.ee_analog_input = std::move(arg);
    return Init_FeedbackState_error_content(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_ee_analog_output
{
public:
  explicit Init_FeedbackState_ee_analog_output(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_ee_analog_input ee_analog_output(::tm_msgs::msg::FeedbackState::_ee_analog_output_type arg)
  {
    msg_.ee_analog_output = std::move(arg);
    return Init_FeedbackState_ee_analog_input(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_ee_digital_input
{
public:
  explicit Init_FeedbackState_ee_digital_input(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_ee_analog_output ee_digital_input(::tm_msgs::msg::FeedbackState::_ee_digital_input_type arg)
  {
    msg_.ee_digital_input = std::move(arg);
    return Init_FeedbackState_ee_analog_output(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_ee_digital_output
{
public:
  explicit Init_FeedbackState_ee_digital_output(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_ee_digital_input ee_digital_output(::tm_msgs::msg::FeedbackState::_ee_digital_output_type arg)
  {
    msg_.ee_digital_output = std::move(arg);
    return Init_FeedbackState_ee_digital_input(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_cb_analog_input
{
public:
  explicit Init_FeedbackState_cb_analog_input(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_ee_digital_output cb_analog_input(::tm_msgs::msg::FeedbackState::_cb_analog_input_type arg)
  {
    msg_.cb_analog_input = std::move(arg);
    return Init_FeedbackState_ee_digital_output(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_cb_analog_output
{
public:
  explicit Init_FeedbackState_cb_analog_output(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_cb_analog_input cb_analog_output(::tm_msgs::msg::FeedbackState::_cb_analog_output_type arg)
  {
    msg_.cb_analog_output = std::move(arg);
    return Init_FeedbackState_cb_analog_input(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_cb_digital_input
{
public:
  explicit Init_FeedbackState_cb_digital_input(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_cb_analog_output cb_digital_input(::tm_msgs::msg::FeedbackState::_cb_digital_input_type arg)
  {
    msg_.cb_digital_input = std::move(arg);
    return Init_FeedbackState_cb_analog_output(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_cb_digital_output
{
public:
  explicit Init_FeedbackState_cb_digital_output(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_cb_digital_input cb_digital_output(::tm_msgs::msg::FeedbackState::_cb_digital_output_type arg)
  {
    msg_.cb_digital_output = std::move(arg);
    return Init_FeedbackState_cb_digital_input(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_robot_light
{
public:
  explicit Init_FeedbackState_robot_light(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_cb_digital_output robot_light(::tm_msgs::msg::FeedbackState::_robot_light_type arg)
  {
    msg_.robot_light = std::move(arg);
    return Init_FeedbackState_cb_digital_output(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_ma_mode
{
public:
  explicit Init_FeedbackState_ma_mode(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_robot_light ma_mode(::tm_msgs::msg::FeedbackState::_ma_mode_type arg)
  {
    msg_.ma_mode = std::move(arg);
    return Init_FeedbackState_robot_light(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_project_speed
{
public:
  explicit Init_FeedbackState_project_speed(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_ma_mode project_speed(::tm_msgs::msg::FeedbackState::_project_speed_type arg)
  {
    msg_.project_speed = std::move(arg);
    return Init_FeedbackState_ma_mode(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_error_code
{
public:
  explicit Init_FeedbackState_error_code(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_project_speed error_code(::tm_msgs::msg::FeedbackState::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_FeedbackState_project_speed(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_camera_light
{
public:
  explicit Init_FeedbackState_camera_light(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_error_code camera_light(::tm_msgs::msg::FeedbackState::_camera_light_type arg)
  {
    msg_.camera_light = std::move(arg);
    return Init_FeedbackState_error_code(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_e_stop
{
public:
  explicit Init_FeedbackState_e_stop(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_camera_light e_stop(::tm_msgs::msg::FeedbackState::_e_stop_type arg)
  {
    msg_.e_stop = std::move(arg);
    return Init_FeedbackState_camera_light(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_safetyguard_a
{
public:
  explicit Init_FeedbackState_safetyguard_a(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_e_stop safetyguard_a(::tm_msgs::msg::FeedbackState::_safetyguard_a_type arg)
  {
    msg_.safetyguard_a = std::move(arg);
    return Init_FeedbackState_e_stop(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_project_pause
{
public:
  explicit Init_FeedbackState_project_pause(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_safetyguard_a project_pause(::tm_msgs::msg::FeedbackState::_project_pause_type arg)
  {
    msg_.project_pause = std::move(arg);
    return Init_FeedbackState_safetyguard_a(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_project_run
{
public:
  explicit Init_FeedbackState_project_run(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_project_pause project_run(::tm_msgs::msg::FeedbackState::_project_run_type arg)
  {
    msg_.project_run = std::move(arg);
    return Init_FeedbackState_project_pause(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_robot_error
{
public:
  explicit Init_FeedbackState_robot_error(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_project_run robot_error(::tm_msgs::msg::FeedbackState::_robot_error_type arg)
  {
    msg_.robot_error = std::move(arg);
    return Init_FeedbackState_project_run(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_is_data_table_correct
{
public:
  explicit Init_FeedbackState_is_data_table_correct(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_robot_error is_data_table_correct(::tm_msgs::msg::FeedbackState::_is_data_table_correct_type arg)
  {
    msg_.is_data_table_correct = std::move(arg);
    return Init_FeedbackState_robot_error(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_robot_link
{
public:
  explicit Init_FeedbackState_robot_link(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_is_data_table_correct robot_link(::tm_msgs::msg::FeedbackState::_robot_link_type arg)
  {
    msg_.robot_link = std::move(arg);
    return Init_FeedbackState_is_data_table_correct(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_joint_tor_max
{
public:
  explicit Init_FeedbackState_joint_tor_max(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_robot_link joint_tor_max(::tm_msgs::msg::FeedbackState::_joint_tor_max_type arg)
  {
    msg_.joint_tor_max = std::move(arg);
    return Init_FeedbackState_robot_link(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_joint_tor_min
{
public:
  explicit Init_FeedbackState_joint_tor_min(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_joint_tor_max joint_tor_min(::tm_msgs::msg::FeedbackState::_joint_tor_min_type arg)
  {
    msg_.joint_tor_min = std::move(arg);
    return Init_FeedbackState_joint_tor_max(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_joint_tor_average
{
public:
  explicit Init_FeedbackState_joint_tor_average(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_joint_tor_min joint_tor_average(::tm_msgs::msg::FeedbackState::_joint_tor_average_type arg)
  {
    msg_.joint_tor_average = std::move(arg);
    return Init_FeedbackState_joint_tor_min(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tcp_force
{
public:
  explicit Init_FeedbackState_tcp_force(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_joint_tor_average tcp_force(::tm_msgs::msg::FeedbackState::_tcp_force_type arg)
  {
    msg_.tcp_force = std::move(arg);
    return Init_FeedbackState_joint_tor_average(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tcp_speed
{
public:
  explicit Init_FeedbackState_tcp_speed(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tcp_force tcp_speed(::tm_msgs::msg::FeedbackState::_tcp_speed_type arg)
  {
    msg_.tcp_speed = std::move(arg);
    return Init_FeedbackState_tcp_force(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tool_pose
{
public:
  explicit Init_FeedbackState_tool_pose(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tcp_speed tool_pose(::tm_msgs::msg::FeedbackState::_tool_pose_type arg)
  {
    msg_.tool_pose = std::move(arg);
    return Init_FeedbackState_tcp_speed(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tool0_pose
{
public:
  explicit Init_FeedbackState_tool0_pose(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tool_pose tool0_pose(::tm_msgs::msg::FeedbackState::_tool0_pose_type arg)
  {
    msg_.tool0_pose = std::move(arg);
    return Init_FeedbackState_tool_pose(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_joint_tor
{
public:
  explicit Init_FeedbackState_joint_tor(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tool0_pose joint_tor(::tm_msgs::msg::FeedbackState::_joint_tor_type arg)
  {
    msg_.joint_tor = std::move(arg);
    return Init_FeedbackState_tool0_pose(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_joint_vel
{
public:
  explicit Init_FeedbackState_joint_vel(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_joint_tor joint_vel(::tm_msgs::msg::FeedbackState::_joint_vel_type arg)
  {
    msg_.joint_vel = std::move(arg);
    return Init_FeedbackState_joint_tor(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_joint_pos
{
public:
  explicit Init_FeedbackState_joint_pos(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_joint_vel joint_pos(::tm_msgs::msg::FeedbackState::_joint_pos_type arg)
  {
    msg_.joint_pos = std::move(arg);
    return Init_FeedbackState_joint_vel(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_disconnection_times
{
public:
  explicit Init_FeedbackState_disconnection_times(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_joint_pos disconnection_times(::tm_msgs::msg::FeedbackState::_disconnection_times_type arg)
  {
    msg_.disconnection_times = std::move(arg);
    return Init_FeedbackState_joint_pos(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_max_not_connect_in_s
{
public:
  explicit Init_FeedbackState_max_not_connect_in_s(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_disconnection_times max_not_connect_in_s(::tm_msgs::msg::FeedbackState::_max_not_connect_in_s_type arg)
  {
    msg_.max_not_connect_in_s = std::move(arg);
    return Init_FeedbackState_disconnection_times(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tmscript_dataerr
{
public:
  explicit Init_FeedbackState_tmscript_dataerr(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_max_not_connect_in_s tmscript_dataerr(::tm_msgs::msg::FeedbackState::_tmscript_dataerr_type arg)
  {
    msg_.tmscript_dataerr = std::move(arg);
    return Init_FeedbackState_max_not_connect_in_s(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tmsrv_dataerr
{
public:
  explicit Init_FeedbackState_tmsrv_dataerr(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tmscript_dataerr tmsrv_dataerr(::tm_msgs::msg::FeedbackState::_tmsrv_dataerr_type arg)
  {
    msg_.tmsrv_dataerr = std::move(arg);
    return Init_FeedbackState_tmscript_dataerr(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tmscript_cperr
{
public:
  explicit Init_FeedbackState_tmscript_cperr(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tmsrv_dataerr tmscript_cperr(::tm_msgs::msg::FeedbackState::_tmscript_cperr_type arg)
  {
    msg_.tmscript_cperr = std::move(arg);
    return Init_FeedbackState_tmsrv_dataerr(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_tmsrv_cperr
{
public:
  explicit Init_FeedbackState_tmsrv_cperr(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tmscript_cperr tmsrv_cperr(::tm_msgs::msg::FeedbackState::_tmsrv_cperr_type arg)
  {
    msg_.tmsrv_cperr = std::move(arg);
    return Init_FeedbackState_tmscript_cperr(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_is_sct_connected
{
public:
  explicit Init_FeedbackState_is_sct_connected(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_tmsrv_cperr is_sct_connected(::tm_msgs::msg::FeedbackState::_is_sct_connected_type arg)
  {
    msg_.is_sct_connected = std::move(arg);
    return Init_FeedbackState_tmsrv_cperr(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_is_svr_connected
{
public:
  explicit Init_FeedbackState_is_svr_connected(::tm_msgs::msg::FeedbackState & msg)
  : msg_(msg)
  {}
  Init_FeedbackState_is_sct_connected is_svr_connected(::tm_msgs::msg::FeedbackState::_is_svr_connected_type arg)
  {
    msg_.is_svr_connected = std::move(arg);
    return Init_FeedbackState_is_sct_connected(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

class Init_FeedbackState_header
{
public:
  Init_FeedbackState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FeedbackState_is_svr_connected header(::tm_msgs::msg::FeedbackState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FeedbackState_is_svr_connected(msg_);
  }

private:
  ::tm_msgs::msg::FeedbackState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_msgs::msg::FeedbackState>()
{
  return tm_msgs::msg::builder::Init_FeedbackState_header();
}

}  // namespace tm_msgs

#endif  // TM_MSGS__MSG__DETAIL__FEEDBACK_STATE__BUILDER_HPP_
