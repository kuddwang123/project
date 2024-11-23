# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/DownRay.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class DownRay(genpy.Message):
  _md5sum = "2128dbaa01dbb390920d49a2d6291353"
  _type = "hj_interface/DownRay"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time timestamp
uint8 left_down   #!< 左下视红外， 0: 有遮挡, 1: 无遮挡.
uint8 right_down  #!< 右下视红外， 0: 有遮挡, 1: 无遮挡."""
  __slots__ = ['timestamp','left_down','right_down']
  _slot_types = ['time','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,left_down,right_down

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DownRay, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.left_down is None:
        self.left_down = 0
      if self.right_down is None:
        self.right_down = 0
    else:
      self.timestamp = genpy.Time()
      self.left_down = 0
      self.right_down = 0

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
      buff.write(_get_struct_2I2B().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.left_down, _x.right_down))
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
      end += 10
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.left_down, _x.right_down,) = _get_struct_2I2B().unpack(str[start:end])
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
      buff.write(_get_struct_2I2B().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.left_down, _x.right_down))
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
      end += 10
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.left_down, _x.right_down,) = _get_struct_2I2B().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I2B = None
def _get_struct_2I2B():
    global _struct_2I2B
    if _struct_2I2B is None:
        _struct_2I2B = struct.Struct("<2I2B")
    return _struct_2I2B