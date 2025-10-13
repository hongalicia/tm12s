// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:msg/SctResponse.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__MSG__DETAIL__SCT_RESPONSE__TRAITS_HPP_
#define TM_MSGS__MSG__DETAIL__SCT_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_msgs/msg/detail/sct_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SctResponse & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: script
  {
    out << "script: ";
    rosidl_generator_traits::value_to_yaml(msg.script, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SctResponse & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: script
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "script: ";
    rosidl_generator_traits::value_to_yaml(msg.script, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SctResponse & msg, bool use_flow_style = false)
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
  const tm_msgs::msg::SctResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::msg::SctResponse & msg)
{
  return tm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::msg::SctResponse>()
{
  return "tm_msgs::msg::SctResponse";
}

template<>
inline const char * name<tm_msgs::msg::SctResponse>()
{
  return "tm_msgs/msg/SctResponse";
}

template<>
struct has_fixed_size<tm_msgs::msg::SctResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::msg::SctResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::msg::SctResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__MSG__DETAIL__SCT_RESPONSE__TRAITS_HPP_
