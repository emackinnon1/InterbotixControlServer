// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__rosidl_typesupport_fastrtps_cpp.hpp"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace interbotix_rpi_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interbotix_rpi_msgs
cdr_serialize(
  const interbotix_rpi_msgs::msg::PixelCommands & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: cmd_type
  cdr << ros_message.cmd_type;
  // Member: set_all_leds
  cdr << (ros_message.set_all_leds ? true : false);
  // Member: pixel
  cdr << ros_message.pixel;
  // Member: color
  cdr << ros_message.color;
  // Member: brightness
  cdr << ros_message.brightness;
  // Member: period
  cdr << ros_message.period;
  // Member: iterations
  cdr << ros_message.iterations;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interbotix_rpi_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  interbotix_rpi_msgs::msg::PixelCommands & ros_message)
{
  // Member: cmd_type
  cdr >> ros_message.cmd_type;

  // Member: set_all_leds
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.set_all_leds = tmp ? true : false;
  }

  // Member: pixel
  cdr >> ros_message.pixel;

  // Member: color
  cdr >> ros_message.color;

  // Member: brightness
  cdr >> ros_message.brightness;

  // Member: period
  cdr >> ros_message.period;

  // Member: iterations
  cdr >> ros_message.iterations;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interbotix_rpi_msgs
get_serialized_size(
  const interbotix_rpi_msgs::msg::PixelCommands & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: cmd_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.cmd_type.size() + 1);
  // Member: set_all_leds
  {
    size_t item_size = sizeof(ros_message.set_all_leds);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pixel
  {
    size_t item_size = sizeof(ros_message.pixel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: color
  {
    size_t item_size = sizeof(ros_message.color);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: brightness
  {
    size_t item_size = sizeof(ros_message.brightness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: period
  {
    size_t item_size = sizeof(ros_message.period);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iterations
  {
    size_t item_size = sizeof(ros_message.iterations);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interbotix_rpi_msgs
max_serialized_size_PixelCommands(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: cmd_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: set_all_leds
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pixel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: color
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: brightness
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: period
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: iterations
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = interbotix_rpi_msgs::msg::PixelCommands;
    is_plain =
      (
      offsetof(DataType, iterations) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PixelCommands__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const interbotix_rpi_msgs::msg::PixelCommands *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PixelCommands__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<interbotix_rpi_msgs::msg::PixelCommands *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PixelCommands__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const interbotix_rpi_msgs::msg::PixelCommands *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PixelCommands__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PixelCommands(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PixelCommands__callbacks = {
  "interbotix_rpi_msgs::msg",
  "PixelCommands",
  _PixelCommands__cdr_serialize,
  _PixelCommands__cdr_deserialize,
  _PixelCommands__get_serialized_size,
  _PixelCommands__max_serialized_size
};

static rosidl_message_type_support_t _PixelCommands__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PixelCommands__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace interbotix_rpi_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_interbotix_rpi_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<interbotix_rpi_msgs::msg::PixelCommands>()
{
  return &interbotix_rpi_msgs::msg::typesupport_fastrtps_cpp::_PixelCommands__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interbotix_rpi_msgs, msg, PixelCommands)() {
  return &interbotix_rpi_msgs::msg::typesupport_fastrtps_cpp::_PixelCommands__handle;
}

#ifdef __cplusplus
}
#endif
