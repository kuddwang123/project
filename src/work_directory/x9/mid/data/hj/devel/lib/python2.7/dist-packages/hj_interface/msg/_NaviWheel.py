# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/NaviWheel.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class NaviWheel(genpy.Message):
  _md5sum = "380645d5befdfd6fee27d39dc348a380"
  _type = "hj_interface/NaviWheel"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time timestamp
int16 left_wheel #!< mm/s
int16 right_wheel"""
  __slots__ = ['timestamp','left_wheel','right_wheel']
  _slot_types = ['time','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,left_wheel,right_wheel

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NaviWheel, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.left_wheel is None:
        self.left_wheel = 0
      if self.right_wheel is None:
        self.right_wheel = 0
    else:
      self.timestamp = genpy.Time()
      self.left_wheel = 0
      self.right_wheel = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2I2h().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.left_wheel, _x.right_wheel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.left_wheel, _x.right_wheel,) = _get_struct_2I2h().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2I2h().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.left_wheel, _x.right_wheel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.left_wheel, _x.right_wheel,) = _get_struct_2I2h().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I2h = None
def _get_struct_2I2h():
    global _struct_2I2h
    if _struct_2I2h is None:
        _struct_2I2h = struct.Struct("<2I2h")
    return _struct_2I2h