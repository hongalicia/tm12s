// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:msg/FeedbackState.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__MSG__DETAIL__FEEDBACK_STATE__TRAITS_HPP_
#define TM_MSGS__MSG__DETAIL__FEEDBACK_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_msgs/msg/detail/feedback_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FeedbackState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_svr_connected
  {
    out << "is_svr_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_svr_connected, out);
    out << ", ";
  }

  // member: is_sct_connected
  {
    out << "is_sct_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_sct_connected, out);
    out << ", ";
  }

  // member: tmsrv_cperr
  {
    out << "tmsrv_cperr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmsrv_cperr, out);
    out << ", ";
  }

  // member: tmscript_cperr
  {
    out << "tmscript_cperr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmscript_cperr, out);
    out << ", ";
  }

  // member: tmsrv_dataerr
  {
    out << "tmsrv_dataerr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmsrv_dataerr, out);
    out << ", ";
  }

  // member: tmscript_dataerr
  {
    out << "tmscript_dataerr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmscript_dataerr, out);
    out << ", ";
  }

  // member: max_not_connect_in_s
  {
    out << "max_not_connect_in_s: ";
    rosidl_generator_traits::value_to_yaml(msg.max_not_connect_in_s, out);
    out << ", ";
  }

  // member: disconnection_times
  {
    out << "disconnection_times: ";
    rosidl_generator_traits::value_to_yaml(msg.disconnection_times, out);
    out << ", ";
  }

  // member: joint_pos
  {
    if (msg.joint_pos.size() == 0) {
      out << "joint_pos: []";
    } else {
      out << "joint_pos: [";
      size_t pending_items = msg.joint_pos.size();
      for (auto item : msg.joint_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_vel
  {
    if (msg.joint_vel.size() == 0) {
      out << "joint_vel: []";
    } else {
      out << "joint_vel: [";
      size_t pending_items = msg.joint_vel.size();
      for (auto item : msg.joint_vel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_tor
  {
    if (msg.joint_tor.size() == 0) {
      out << "joint_tor: []";
    } else {
      out << "joint_tor: [";
      size_t pending_items = msg.joint_tor.size();
      for (auto item : msg.joint_tor) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tool0_pose
  {
    if (msg.tool0_pose.size() == 0) {
      out << "tool0_pose: []";
    } else {
      out << "tool0_pose: [";
      size_t pending_items = msg.tool0_pose.size();
      for (auto item : msg.tool0_pose) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tool_pose
  {
    if (msg.tool_pose.size() == 0) {
      out << "tool_pose: []";
    } else {
      out << "tool_pose: [";
      size_t pending_items = msg.tool_pose.size();
      for (auto item : msg.tool_pose) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tcp_speed
  {
    if (msg.tcp_speed.size() == 0) {
      out << "tcp_speed: []";
    } else {
      out << "tcp_speed: [";
      size_t pending_items = msg.tcp_speed.size();
      for (auto item : msg.tcp_speed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tcp_force
  {
    if (msg.tcp_force.size() == 0) {
      out << "tcp_force: []";
    } else {
      out << "tcp_force: [";
      size_t pending_items = msg.tcp_force.size();
      for (auto item : msg.tcp_force) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_tor_average
  {
    if (msg.joint_tor_average.size() == 0) {
      out << "joint_tor_average: []";
    } else {
      out << "joint_tor_average: [";
      size_t pending_items = msg.joint_tor_average.size();
      for (auto item : msg.joint_tor_average) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_tor_min
  {
    if (msg.joint_tor_min.size() == 0) {
      out << "joint_tor_min: []";
    } else {
      out << "joint_tor_min: [";
      size_t pending_items = msg.joint_tor_min.size();
      for (auto item : msg.joint_tor_min) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_tor_max
  {
    if (msg.joint_tor_max.size() == 0) {
      out << "joint_tor_max: []";
    } else {
      out << "joint_tor_max: [";
      size_t pending_items = msg.joint_tor_max.size();
      for (auto item : msg.joint_tor_max) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_link
  {
    out << "robot_link: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_link, out);
    out << ", ";
  }

  // member: is_data_table_correct
  {
    out << "is_data_table_correct: ";
    rosidl_generator_traits::value_to_yaml(msg.is_data_table_correct, out);
    out << ", ";
  }

  // member: robot_error
  {
    out << "robot_error: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_error, out);
    out << ", ";
  }

  // member: project_run
  {
    out << "project_run: ";
    rosidl_generator_traits::value_to_yaml(msg.project_run, out);
    out << ", ";
  }

  // member: project_pause
  {
    out << "project_pause: ";
    rosidl_generator_traits::value_to_yaml(msg.project_pause, out);
    out << ", ";
  }

  // member: safetyguard_a
  {
    out << "safetyguard_a: ";
    rosidl_generator_traits::value_to_yaml(msg.safetyguard_a, out);
    out << ", ";
  }

  // member: e_stop
  {
    out << "e_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.e_stop, out);
    out << ", ";
  }

  // member: camera_light
  {
    out << "camera_light: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_light, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: project_speed
  {
    out << "project_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.project_speed, out);
    out << ", ";
  }

  // member: ma_mode
  {
    out << "ma_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.ma_mode, out);
    out << ", ";
  }

  // member: robot_light
  {
    out << "robot_light: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_light, out);
    out << ", ";
  }

  // member: cb_digital_output
  {
    if (msg.cb_digital_output.size() == 0) {
      out << "cb_digital_output: []";
    } else {
      out << "cb_digital_output: [";
      size_t pending_items = msg.cb_digital_output.size();
      for (auto item : msg.cb_digital_output) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cb_digital_input
  {
    if (msg.cb_digital_input.size() == 0) {
      out << "cb_digital_input: []";
    } else {
      out << "cb_digital_input: [";
      size_t pending_items = msg.cb_digital_input.size();
      for (auto item : msg.cb_digital_input) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cb_analog_output
  {
    if (msg.cb_analog_output.size() == 0) {
      out << "cb_analog_output: []";
    } else {
      out << "cb_analog_output: [";
      size_t pending_items = msg.cb_analog_output.size();
      for (auto item : msg.cb_analog_output) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cb_analog_input
  {
    if (msg.cb_analog_input.size() == 0) {
      out << "cb_analog_input: []";
    } else {
      out << "cb_analog_input: [";
      size_t pending_items = msg.cb_analog_input.size();
      for (auto item : msg.cb_analog_input) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_digital_output
  {
    if (msg.ee_digital_output.size() == 0) {
      out << "ee_digital_output: []";
    } else {
      out << "ee_digital_output: [";
      size_t pending_items = msg.ee_digital_output.size();
      for (auto item : msg.ee_digital_output) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_digital_input
  {
    if (msg.ee_digital_input.size() == 0) {
      out << "ee_digital_input: []";
    } else {
      out << "ee_digital_input: [";
      size_t pending_items = msg.ee_digital_input.size();
      for (auto item : msg.ee_digital_input) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_analog_output
  {
    if (msg.ee_analog_output.size() == 0) {
      out << "ee_analog_output: []";
    } else {
      out << "ee_analog_output: [";
      size_t pending_items = msg.ee_analog_output.size();
      for (auto item : msg.ee_analog_output) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ee_analog_input
  {
    if (msg.ee_analog_input.size() == 0) {
      out << "ee_analog_input: []";
    } else {
      out << "ee_analog_input: [";
      size_t pending_items = msg.ee_analog_input.size();
      for (auto item : msg.ee_analog_input) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: error_content
  {
    out << "error_content: ";
    rosidl_generator_traits::value_to_yaml(msg.error_content, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FeedbackState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: is_svr_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_svr_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_svr_connected, out);
    out << "\n";
  }

  // member: is_sct_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_sct_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_sct_connected, out);
    out << "\n";
  }

  // member: tmsrv_cperr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tmsrv_cperr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmsrv_cperr, out);
    out << "\n";
  }

  // member: tmscript_cperr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tmscript_cperr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmscript_cperr, out);
    out << "\n";
  }

  // member: tmsrv_dataerr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tmsrv_dataerr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmsrv_dataerr, out);
    out << "\n";
  }

  // member: tmscript_dataerr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tmscript_dataerr: ";
    rosidl_generator_traits::value_to_yaml(msg.tmscript_dataerr, out);
    out << "\n";
  }

  // member: max_not_connect_in_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_not_connect_in_s: ";
    rosidl_generator_traits::value_to_yaml(msg.max_not_connect_in_s, out);
    out << "\n";
  }

  // member: disconnection_times
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "disconnection_times: ";
    rosidl_generator_traits::value_to_yaml(msg.disconnection_times, out);
    out << "\n";
  }

  // member: joint_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_pos.size() == 0) {
      out << "joint_pos: []\n";
    } else {
      out << "joint_pos:\n";
      for (auto item : msg.joint_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_vel.size() == 0) {
      out << "joint_vel: []\n";
    } else {
      out << "joint_vel:\n";
      for (auto item : msg.joint_vel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_tor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_tor.size() == 0) {
      out << "joint_tor: []\n";
    } else {
      out << "joint_tor:\n";
      for (auto item : msg.joint_tor) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tool0_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tool0_pose.size() == 0) {
      out << "tool0_pose: []\n";
    } else {
      out << "tool0_pose:\n";
      for (auto item : msg.tool0_pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tool_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tool_pose.size() == 0) {
      out << "tool_pose: []\n";
    } else {
      out << "tool_pose:\n";
      for (auto item : msg.tool_pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tcp_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tcp_speed.size() == 0) {
      out << "tcp_speed: []\n";
    } else {
      out << "tcp_speed:\n";
      for (auto item : msg.tcp_speed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tcp_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tcp_force.size() == 0) {
      out << "tcp_force: []\n";
    } else {
      out << "tcp_force:\n";
      for (auto item : msg.tcp_force) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_tor_average
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_tor_average.size() == 0) {
      out << "joint_tor_average: []\n";
    } else {
      out << "joint_tor_average:\n";
      for (auto item : msg.joint_tor_average) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_tor_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_tor_min.size() == 0) {
      out << "joint_tor_min: []\n";
    } else {
      out << "joint_tor_min:\n";
      for (auto item : msg.joint_tor_min) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_tor_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_tor_max.size() == 0) {
      out << "joint_tor_max: []\n";
    } else {
      out << "joint_tor_max:\n";
      for (auto item : msg.joint_tor_max) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: robot_link
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_link: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_link, out);
    out << "\n";
  }

  // member: is_data_table_correct
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_data_table_correct: ";
    rosidl_generator_traits::value_to_yaml(msg.is_data_table_correct, out);
    out << "\n";
  }

  // member: robot_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_error: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_error, out);
    out << "\n";
  }

  // member: project_run
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "project_run: ";
    rosidl_generator_traits::value_to_yaml(msg.project_run, out);
    out << "\n";
  }

  // member: project_pause
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "project_pause: ";
    rosidl_generator_traits::value_to_yaml(msg.project_pause, out);
    out << "\n";
  }

  // member: safetyguard_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safetyguard_a: ";
    rosidl_generator_traits::value_to_yaml(msg.safetyguard_a, out);
    out << "\n";
  }

  // member: e_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.e_stop, out);
    out << "\n";
  }

  // member: camera_light
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_light: ";
    rosidl_generator_traits::value_to_yaml(msg.camera_light, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: project_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "project_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.project_speed, out);
    out << "\n";
  }

  // member: ma_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ma_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.ma_mode, out);
    out << "\n";
  }

  // member: robot_light
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_light: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_light, out);
    out << "\n";
  }

  // member: cb_digital_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cb_digital_output.size() == 0) {
      out << "cb_digital_output: []\n";
    } else {
      out << "cb_digital_output:\n";
      for (auto item : msg.cb_digital_output) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cb_digital_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cb_digital_input.size() == 0) {
      out << "cb_digital_input: []\n";
    } else {
      out << "cb_digital_input:\n";
      for (auto item : msg.cb_digital_input) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cb_analog_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cb_analog_output.size() == 0) {
      out << "cb_analog_output: []\n";
    } else {
      out << "cb_analog_output:\n";
      for (auto item : msg.cb_analog_output) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cb_analog_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cb_analog_input.size() == 0) {
      out << "cb_analog_input: []\n";
    } else {
      out << "cb_analog_input:\n";
      for (auto item : msg.cb_analog_input) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_digital_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_digital_output.size() == 0) {
      out << "ee_digital_output: []\n";
    } else {
      out << "ee_digital_output:\n";
      for (auto item : msg.ee_digital_output) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_digital_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_digital_input.size() == 0) {
      out << "ee_digital_input: []\n";
    } else {
      out << "ee_digital_input:\n";
      for (auto item : msg.ee_digital_input) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_analog_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_analog_output.size() == 0) {
      out << "ee_analog_output: []\n";
    } else {
      out << "ee_analog_output:\n";
      for (auto item : msg.ee_analog_output) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ee_analog_input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ee_analog_input.size() == 0) {
      out << "ee_analog_input: []\n";
    } else {
      out << "ee_analog_input:\n";
      for (auto item : msg.ee_analog_input) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: error_content
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_content: ";
    rosidl_generator_traits::value_to_yaml(msg.error_content, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FeedbackState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tm_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_msgs::msg::FeedbackState & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::msg::FeedbackState & msg)
{
  return tm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::msg::FeedbackState>()
{
  return "tm_msgs::msg::FeedbackState";
}

template<>
inline const char * name<tm_msgs::msg::FeedbackState>()
{
  return "tm_msgs/msg/FeedbackState";
}

template<>
struct has_fixed_size<tm_msgs::msg::FeedbackState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::msg::FeedbackState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::msg::FeedbackState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__MSG__DETAIL__FEEDBACK_STATE__TRAITS_HPP_
