// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interbotix_rpi_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.h"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // cmd_type
#include "rosidl_runtime_c/string_functions.h"  // cmd_type

// forward declare type support functions


using _PixelCommands__ros_msg_type = interbotix_rpi_msgs__msg__PixelCommands;

static bool _PixelCommands__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PixelCommands__ros_msg_type * ros_message = static_cast<const _PixelCommands__ros_msg_type *>(untyped_ros_message);
  // Field name: cmd_type
  {
    const rosidl_runtime_c__String * str = &ros_message->cmd_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: set_all_leds
  {
    cdr << (ros_message->set_all_leds ? true : false);
  }

  // Field name: pixel
  {
    cdr << ros_message->pixel;
  }

  // Field name: color
  {
    cdr << ros_message->color;
  }

  // Field name: brightness
  {
    cdr << ros_message->brightness;
  }

  // Field name: period
  {
    cdr << ros_message->period;
  }

  // Field name: iterations
  {
    cdr << ros_message->iterations;
  }

  return true;
}

static bool _PixelCommands__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PixelCommands__ros_msg_type * ros_message = static_cast<_PixelCommands__ros_msg_type *>(untyped_ros_message);
  // Field name: cmd_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->cmd_type.data) {
      rosidl_runtime_c__String__init(&ros_message->cmd_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->cmd_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'cmd_type'\n");
      return false;
    }
  }

  // Field name: set_all_leds
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->set_all_leds = tmp ? true : false;
  }

  // Field name: pixel
  {
    cdr >> ros_message->pixel;
  }

  // Field name: color
  {
    cdr >> ros_message->color;
  }

  // Field name: brightness
  {
    cdr >> ros_message->brightness;
  }

  // Field name: period
  {
    cdr >> ros_message->period;
  }

  // Field name: iterations
  {
    cdr >> ros_message->iterations;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interbotix_rpi_msgs
size_t get_serialized_size_interbotix_rpi_msgs__msg__PixelCommands(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PixelCommands__ros_msg_type * ros_message = static_cast<const _PixelCommands__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name cmd_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->cmd_type.size + 1);
  // field.name set_all_leds
  {
    size_t item_size = sizeof(ros_message->set_all_leds);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pixel
  {
    size_t item_size = sizeof(ros_message->pixel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name color
  {
    size_t item_size = sizeof(ros_message->color);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name brightness
  {
    size_t item_size = sizeof(ros_message->brightness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name period
  {
    size_t item_size = sizeof(ros_message->period);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iterations
  {
    size_t item_size = sizeof(ros_message->iterations);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PixelCommands__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interbotix_rpi_msgs__msg__PixelCommands(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interbotix_rpi_msgs
size_t max_serialized_size_interbotix_rpi_msgs__msg__PixelCommands(
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

  // member: cmd_type
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
  // member: set_all_leds
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pixel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: color
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: brightness
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: period
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: iterations
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
    using DataType = interbotix_rpi_msgs__msg__PixelCommands;
    is_plain =
      (
      offsetof(DataType, iterations) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PixelCommands__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interbotix_rpi_msgs__msg__PixelCommands(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PixelCommands = {
  "interbotix_rpi_msgs::msg",
  "PixelCommands",
  _PixelCommands__cdr_serialize,
  _PixelCommands__cdr_deserialize,
  _PixelCommands__get_serialized_size,
  _PixelCommands__max_serialized_size
};

static rosidl_message_type_support_t _PixelCommands__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PixelCommands,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interbotix_rpi_msgs, msg, PixelCommands)() {
  return &_PixelCommands__type_support;
}

#if defined(__cplusplus)
}
#endif
