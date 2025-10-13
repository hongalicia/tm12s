// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_msgs:msg/SvrResponse.idl
// generated code does not contain a copyright notice

#ifndef TM_MSGS__MSG__DETAIL__SVR_RESPONSE__TRAITS_HPP_
#define TM_MSGS__MSG__DETAIL__SVR_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_msgs/msg/detail/svr_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SvrResponse & msg,
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

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: content
  {
    out << "content: ";
    rosidl_generator_traits::value_to_yaml(msg.content, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SvrResponse & msg,
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

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: content
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "content: ";
    rosidl_generator_traits::value_to_yaml(msg.content, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SvrResponse & msg, bool use_flow_style = false)
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
  const tm_msgs::msg::SvrResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tm_msgs::msg::SvrResponse & msg)
{
  return tm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tm_msgs::msg::SvrResponse>()
{
  return "tm_msgs::msg::SvrResponse";
}

template<>
inline const char * name<tm_msgs::msg::SvrResponse>()
{
  return "tm_msgs/msg/SvrResponse";
}

template<>
struct has_fixed_size<tm_msgs::msg::SvrResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_msgs::msg::SvrResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_msgs::msg::SvrResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TM_MSGS__MSG__DETAIL__SVR_RESPONSE__TRAITS_HPP_
