// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from interbotix_rpi_msgs:msg/PixelCommands.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__struct.h"
#include "interbotix_rpi_msgs/msg/detail/pixel_commands__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool interbotix_rpi_msgs__msg__pixel_commands__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("interbotix_rpi_msgs.msg._pixel_commands.PixelCommands", full_classname_dest, 53) == 0);
  }
  interbotix_rpi_msgs__msg__PixelCommands * ros_message = _ros_message;
  {  // cmd_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->cmd_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // set_all_leds
    PyObject * field = PyObject_GetAttrString(_pymsg, "set_all_leds");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->set_all_leds = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pixel
    PyObject * field = PyObject_GetAttrString(_pymsg, "pixel");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pixel = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // color
    PyObject * field = PyObject_GetAttrString(_pymsg, "color");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->color = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // brightness
    PyObject * field = PyObject_GetAttrString(_pymsg, "brightness");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->brightness = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // period
    PyObject * field = PyObject_GetAttrString(_pymsg, "period");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->period = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // iterations
    PyObject * field = PyObject_GetAttrString(_pymsg, "iterations");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->iterations = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * interbotix_rpi_msgs__msg__pixel_commands__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PixelCommands */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("interbotix_rpi_msgs.msg._pixel_commands");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PixelCommands");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  interbotix_rpi_msgs__msg__PixelCommands * ros_message = (interbotix_rpi_msgs__msg__PixelCommands *)raw_ros_message;
  {  // cmd_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->cmd_type.data,
      strlen(ros_message->cmd_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // set_all_leds
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->set_all_leds ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "set_all_leds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pixel
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pixel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pixel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->color);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // brightness
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->brightness);
    {
      int rc = PyObject_SetAttrString(_pymessage, "brightness", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // period
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->period);
    {
      int rc = PyObject_SetAttrString(_pymessage, "period", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iterations
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->iterations);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iterations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
