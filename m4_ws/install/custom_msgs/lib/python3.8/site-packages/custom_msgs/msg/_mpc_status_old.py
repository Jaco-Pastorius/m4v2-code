# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/MPCStatusOld.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'input'
# Member 'xref'
# Member 'uref'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MPCStatusOld(type):
    """Metaclass of message 'MPCStatusOld'."""

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
            module = import_type_support('custom_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_msgs.msg.MPCStatusOld')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mpc_status_old
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mpc_status_old
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mpc_status_old
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mpc_status_old
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mpc_status_old

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MPCStatusOld(metaclass=Metaclass_MPCStatusOld):
    """Message class 'MPCStatusOld'."""

    __slots__ = [
        '_timestamp',
        '_status',
        '_comptime',
        '_input',
        '_xref',
        '_uref',
        '_x',
        '_y',
        '_z',
        '_thetaz',
        '_thetay',
        '_thetax',
        '_dx',
        '_dy',
        '_dz',
        '_omegax',
        '_omegay',
        '_omegaz',
        '_varphi',
        '_tiltvel',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'status': 'int32',
        'comptime': 'float',
        'input': 'float[4]',
        'xref': 'float[12]',
        'uref': 'float[4]',
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'thetaz': 'float',
        'thetay': 'float',
        'thetax': 'float',
        'dx': 'float',
        'dy': 'float',
        'dz': 'float',
        'omegax': 'float',
        'omegay': 'float',
        'omegaz': 'float',
        'varphi': 'float',
        'tiltvel': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 12),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.status = kwargs.get('status', int())
        self.comptime = kwargs.get('comptime', float())
        if 'input' not in kwargs:
            self.input = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.input = numpy.array(kwargs.get('input'), dtype=numpy.float32)
            assert self.input.shape == (4, )
        if 'xref' not in kwargs:
            self.xref = numpy.zeros(12, dtype=numpy.float32)
        else:
            self.xref = numpy.array(kwargs.get('xref'), dtype=numpy.float32)
            assert self.xref.shape == (12, )
        if 'uref' not in kwargs:
            self.uref = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.uref = numpy.array(kwargs.get('uref'), dtype=numpy.float32)
            assert self.uref.shape == (4, )
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.thetaz = kwargs.get('thetaz', float())
        self.thetay = kwargs.get('thetay', float())
        self.thetax = kwargs.get('thetax', float())
        self.dx = kwargs.get('dx', float())
        self.dy = kwargs.get('dy', float())
        self.dz = kwargs.get('dz', float())
        self.omegax = kwargs.get('omegax', float())
        self.omegay = kwargs.get('omegay', float())
        self.omegaz = kwargs.get('omegaz', float())
        self.varphi = kwargs.get('varphi', float())
        self.tiltvel = kwargs.get('tiltvel', float())

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
        if self.timestamp != other.timestamp:
            return False
        if self.status != other.status:
            return False
        if self.comptime != other.comptime:
            return False
        if all(self.input != other.input):
            return False
        if all(self.xref != other.xref):
            return False
        if all(self.uref != other.uref):
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.thetaz != other.thetaz:
            return False
        if self.thetay != other.thetay:
            return False
        if self.thetax != other.thetax:
            return False
        if self.dx != other.dx:
            return False
        if self.dy != other.dy:
            return False
        if self.dz != other.dz:
            return False
        if self.omegax != other.omegax:
            return False
        if self.omegay != other.omegay:
            return False
        if self.omegaz != other.omegaz:
            return False
        if self.varphi != other.varphi:
            return False
        if self.tiltvel != other.tiltvel:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp = value

    @property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'status' field must be an integer in [-2147483648, 2147483647]"
        self._status = value

    @property
    def comptime(self):
        """Message field 'comptime'."""
        return self._comptime

    @comptime.setter
    def comptime(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'comptime' field must be of type 'float'"
        self._comptime = value

    @property  # noqa: A003
    def input(self):  # noqa: A003
        """Message field 'input'."""
        return self._input

    @input.setter  # noqa: A003
    def input(self, value):  # noqa: A003
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'input' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'input' numpy.ndarray() must have a size of 4"
            self._input = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'input' field must be a set or sequence with length 4 and each value of type 'float'"
        self._input = numpy.array(value, dtype=numpy.float32)

    @property
    def xref(self):
        """Message field 'xref'."""
        return self._xref

    @xref.setter
    def xref(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'xref' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 12, \
                "The 'xref' numpy.ndarray() must have a size of 12"
            self._xref = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 12 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'xref' field must be a set or sequence with length 12 and each value of type 'float'"
        self._xref = numpy.array(value, dtype=numpy.float32)

    @property
    def uref(self):
        """Message field 'uref'."""
        return self._uref

    @uref.setter
    def uref(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'uref' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'uref' numpy.ndarray() must have a size of 4"
            self._uref = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'uref' field must be a set or sequence with length 4 and each value of type 'float'"
        self._uref = numpy.array(value, dtype=numpy.float32)

    @property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
        self._z = value

    @property
    def thetaz(self):
        """Message field 'thetaz'."""
        return self._thetaz

    @thetaz.setter
    def thetaz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetaz' field must be of type 'float'"
        self._thetaz = value

    @property
    def thetay(self):
        """Message field 'thetay'."""
        return self._thetay

    @thetay.setter
    def thetay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetay' field must be of type 'float'"
        self._thetay = value

    @property
    def thetax(self):
        """Message field 'thetax'."""
        return self._thetax

    @thetax.setter
    def thetax(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thetax' field must be of type 'float'"
        self._thetax = value

    @property
    def dx(self):
        """Message field 'dx'."""
        return self._dx

    @dx.setter
    def dx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dx' field must be of type 'float'"
        self._dx = value

    @property
    def dy(self):
        """Message field 'dy'."""
        return self._dy

    @dy.setter
    def dy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dy' field must be of type 'float'"
        self._dy = value

    @property
    def dz(self):
        """Message field 'dz'."""
        return self._dz

    @dz.setter
    def dz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dz' field must be of type 'float'"
        self._dz = value

    @property
    def omegax(self):
        """Message field 'omegax'."""
        return self._omegax

    @omegax.setter
    def omegax(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'omegax' field must be of type 'float'"
        self._omegax = value

    @property
    def omegay(self):
        """Message field 'omegay'."""
        return self._omegay

    @omegay.setter
    def omegay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'omegay' field must be of type 'float'"
        self._omegay = value

    @property
    def omegaz(self):
        """Message field 'omegaz'."""
        return self._omegaz

    @omegaz.setter
    def omegaz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'omegaz' field must be of type 'float'"
        self._omegaz = value

    @property
    def varphi(self):
        """Message field 'varphi'."""
        return self._varphi

    @varphi.setter
    def varphi(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'varphi' field must be of type 'float'"
        self._varphi = value

    @property
    def tiltvel(self):
        """Message field 'tiltvel'."""
        return self._tiltvel

    @tiltvel.setter
    def tiltvel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tiltvel' field must be of type 'float'"
        self._tiltvel = value
