// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolov8_msgs:msg/Yolov8Inference.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_MSGS__MSG__DETAIL__YOLOV8_INFERENCE__TRAITS_HPP_
#define YOLOV8_MSGS__MSG__DETAIL__YOLOV8_INFERENCE__TRAITS_HPP_

#include "yolov8_msgs/msg/detail/yolov8_inference__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolov8_msgs::msg::Yolov8Inference>()
{
  return "yolov8_msgs::msg::Yolov8Inference";
}

template<>
inline const char * name<yolov8_msgs::msg::Yolov8Inference>()
{
  return "yolov8_msgs/msg/Yolov8Inference";
}

template<>
struct has_fixed_size<yolov8_msgs::msg::Yolov8Inference>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolov8_msgs::msg::Yolov8Inference>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolov8_msgs::msg::Yolov8Inference>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLOV8_MSGS__MSG__DETAIL__YOLOV8_INFERENCE__TRAITS_HPP_
