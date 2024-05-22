# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/MPCStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'x'
# Member 'xref'
# Member 'xnext'
# Member 'u'
# Member 'uref'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MPCStatus(type):
    """Metaclass of message 'MPCStatus'."""

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
                'custom_msgs.msg.MPCStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mpc_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mpc_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mpc_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mpc_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mpc_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MPCStatus(metaclass=Metaclass_MPCStatus):
    """Message class 'MPCStatus'."""

    __slots__ = [
        '_timestamp',
        '_x',
        '_xref',
        '_xnext',
        '_u',
        '_uref',
        '_varphi',
        '_tiltvel',
        '_status',
        '_trackingdone',
        '_grounded',
        '_comptime',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'x': 'float[12]',
        'xref': 'float[12]',
        'xnext': 'float[12]',
        'u': 'float[4]',
        'uref': 'float[4]',
        'varphi': 'float',
        'tiltvel': 'float',
        'status': 'int32',
        'trackingdone': 'int32',
        'grounded': 'int32',
        'comptime': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 12),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 12),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 12),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        if 'x' not in kwargs:
            self.x = numpy.zeros(12, dtype=numpy.float32)
        else:
            self.x = numpy.array(kwargs.get('x'), dtype=numpy.float32)
            assert self.x.shape == (12, )
        if 'xref' not in kwargs:
            self.xref = numpy.zeros(12, dtype=numpy.float32)
        else:
            self.xref = numpy.array(kwargs.get('xref'), dtype=numpy.float32)
            assert self.xref.shape == (12, )
        if 'xnext' not in kwargs:
            self.xnext = numpy.zeros(12, dtype=numpy.float32)
        else:
            self.xnext = numpy.array(kwargs.get('xnext'), dtype=numpy.float32)
            assert self.xnext.shape == (12, )
        if 'u' not in kwargs:
            self.u = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.u = numpy.array(kwargs.get('u'), dtype=numpy.float32)
            assert self.u.shape == (4, )
        if 'uref' not in kwargs:
            self.uref = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.uref = numpy.array(kwargs.get('uref'), dtype=numpy.float32)
            assert self.uref.shape == (4, )
        self.varphi = kwargs.get('varphi', float())
        self.tiltvel = kwargs.get('tiltvel', float())
        self.status = kwargs.get('status', int())
        self.trackingdone = kwargs.get('trackingdone', int())
        self.grounded = kwargs.get('grounded', int())
        self.comptime = kwargs.get('comptime', float())

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
        if all(self.x != other.x):
            return False
        if all(self.xref != other.xref):
            return False
        if all(self.xnext != other.xnext):
            return False
        if all(self.u != other.u):
            return False
        if all(self.uref != other.uref):
            return False
        if self.varphi != other.varphi:
            return False
        if self.tiltvel != other.tiltvel:
            return False
        if self.status != other.status:
            return False
        if self.trackingdone != other.trackingdone:
            return False
        if self.grounded != other.grounded:
            return False
        if self.comptime != other.comptime:
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
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'x' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 12, \
                "The 'x' numpy.ndarray() must have a size of 12"
            self._x = value
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
                "The 'x' field must be a set or sequence with length 12 and each value of type 'float'"
        self._x = numpy.array(value, dtype=numpy.float32)

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
    def xnext(self):
        """Message field 'xnext'."""
        return self._xnext

    @xnext.setter
    def xnext(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'xnext' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 12, \
                "The 'xnext' numpy.ndarray() must have a size of 12"
            self._xnext = value
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
                "The 'xnext' field must be a set or sequence with length 12 and each value of type 'float'"
        self._xnext = numpy.array(value, dtype=numpy.float32)

    @property
    def u(self):
        """Message field 'u'."""
        return self._u

    @u.setter
    def u(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'u' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'u' numpy.ndarray() must have a size of 4"
            self._u = value
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
                "The 'u' field must be a set or sequence with length 4 and each value of type 'float'"
        self._u = numpy.array(value, dtype=numpy.float32)

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
    def trackingdone(self):
        """Message field 'trackingdone'."""
        return self._trackingdone

    @trackingdone.setter
    def trackingdone(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trackingdone' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'trackingdone' field must be an integer in [-2147483648, 2147483647]"
        self._trackingdone = value

    @property
    def grounded(self):
        """Message field 'grounded'."""
        return self._grounded

    @grounded.setter
    def grounded(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'grounded' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'grounded' field must be an integer in [-2147483648, 2147483647]"
        self._grounded = value

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
