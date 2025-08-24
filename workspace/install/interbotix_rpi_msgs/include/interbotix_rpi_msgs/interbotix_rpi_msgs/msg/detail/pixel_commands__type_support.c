// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__rosidl_typesupport_introspection_c.h"
#include "interbotix_rpi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__functions.h"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.h"


// Include directives for member types
// Member `cmd_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interbotix_rpi_msgs__msg__PixelCommands__init(message_memory);
}

void interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_fini_function(void * message_memory)
{
  interbotix_rpi_msgs__msg__PixelCommands__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_member_array[7] = {
  {
    "cmd_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, cmd_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "set_all_leds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, set_all_leds),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pixel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, pixel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "color",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, color),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "brightness",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, brightness),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "period",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, period),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "iterations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interbotix_rpi_msgs__msg__PixelCommands, iterations),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_members = {
  "interbotix_rpi_msgs__msg",  // message namespace
  "PixelCommands",  // message name
  7,  // number of fields
  sizeof(interbotix_rpi_msgs__msg__PixelCommands),
  interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_member_array,  // message members
  interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_init_function,  // function to initialize message memory (memory has to be allocated)
  interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_type_support_handle = {
  0,
  &interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interbotix_rpi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interbotix_rpi_msgs, msg, PixelCommands)() {
  if (!interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_type_support_handle.typesupport_identifier) {
    interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interbotix_rpi_msgs__msg__PixelCommands__rosidl_typesupport_introspection_c__PixelCommands_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
