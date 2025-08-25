# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interbotix_rpi_msgs:msg/PixelCommands.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PixelCommands(type):
    """Metaclass of message 'PixelCommands'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interbotix_rpi_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interbotix_rpi_msgs.msg.PixelCommands')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pixel_commands
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pixel_commands
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pixel_commands
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pixel_commands
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pixel_commands

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PixelCommands(metaclass=Metaclass_PixelCommands):
    """Message class 'PixelCommands'."""

    __slots__ = [
        '_cmd_type',
        '_set_all_leds',
        '_pixel',
        '_color',
        '_brightness',
        '_period',
        '_iterations',
    ]

    _fields_and_field_types = {
        'cmd_type': 'string',
        'set_all_leds': 'boolean',
        'pixel': 'uint8',
        'color': 'uint32',
        'brightness': 'uint8',
        'period': 'uint32',
        'iterations': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd_type = kwargs.get('cmd_type', str())
        self.set_all_leds = kwargs.get('set_all_leds', bool())
        self.pixel = kwargs.get('pixel', int())
        self.color = kwargs.get('color', int())
        self.brightness = kwargs.get('brightness', int())
        self.period = kwargs.get('period', int())
        self.iterations = kwargs.get('iterations', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.cmd_type != other.cmd_type:
            return False
        if self.set_all_leds != other.set_all_leds:
            return False
        if self.pixel != other.pixel:
            return False
        if self.color != other.color:
            return False
        if self.brightness != other.brightness:
            return False
        if self.period != other.period:
            return False
        if self.iterations != other.iterations:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd_type(self):
        """Message field 'cmd_type'."""
        return self._cmd_type

    @cmd_type.setter
    def cmd_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'cmd_type' field must be of type 'str'"
        self._cmd_type = value

    @builtins.property
    def set_all_leds(self):
        """Message field 'set_all_leds'."""
        return self._set_all_leds

    @set_all_leds.setter
    def set_all_leds(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'set_all_leds' field must be of type 'bool'"
        self._set_all_leds = value

    @builtins.property
    def pixel(self):
        """Message field 'pixel'."""
        return self._pixel

    @pixel.setter
    def pixel(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pixel' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'pixel' field must be an unsigned integer in [0, 255]"
        self._pixel = value

    @builtins.property
    def color(self):
        """Message field 'color'."""
        return self._color

    @color.setter
    def color(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'color' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'color' field must be an unsigned integer in [0, 4294967295]"
        self._color = value

    @builtins.property
    def brightness(self):
        """Message field 'brightness'."""
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'brightness' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'brightness' field must be an unsigned integer in [0, 255]"
        self._brightness = value

    @builtins.property
    def period(self):
        """Message field 'period'."""
        return self._period

    @period.setter
    def period(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'period' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'period' field must be an unsigned integer in [0, 4294967295]"
        self._period = value

    @builtins.property
    def iterations(self):
        """Message field 'iterations'."""
        return self._iterations

    @iterations.setter
    def iterations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'iterations' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'iterations' field must be an unsigned integer in [0, 255]"
        self._iterations = value
