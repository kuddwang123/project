# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/Nav.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class Nav(genpy.Message):
  _md5sum = "fd67e180883d4e466346385259ba6618"
  _type = "hj_interface/Nav"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time custom_time
float32 left_msg
float32 right_msg
float32 pump_a
float32 pump_b"""
  __slots__ = ['custom_time','left_msg','right_msg','pump_a','pump_b']
  _slot_types = ['time','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       custom_time,left_msg,right_msg,pump_a,pump_b

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Nav, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.custom_time is None:
        self.custom_time = genpy.Time()
      if self.left_msg is None:
        self.left_msg = 0.
      if self.right_msg is None:
        self.right_msg = 0.
      if self.pump_a is None:
        self.pump_a = 0.
      if self.pump_b is None:
        self.pump_b = 0.
    else:
      self.custom_time = genpy.Time()
      self.left_msg = 0.
      self.right_msg = 0.
      self.pump_a = 0.
      self.pump_b = 0.

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
      buff.write(_get_struct_2I4f().pack(_x.custom_time.secs, _x.custom_time.nsecs, _x.left_msg, _x.right_msg, _x.pump_a, _x.pump_b))
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
      if self.custom_time is None:
        self.custom_time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.custom_time.secs, _x.custom_time.nsecs, _x.left_msg, _x.right_msg, _x.pump_a, _x.pump_b,) = _get_struct_2I4f().unpack(str[start:end])
      self.custom_time.canon()
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
      buff.write(_get_struct_2I4f().pack(_x.custom_time.secs, _x.custom_time.nsecs, _x.left_msg, _x.right_msg, _x.pump_a, _x.pump_b))
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
      if self.custom_time is None:
        self.custom_time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.custom_time.secs, _x.custom_time.nsecs, _x.left_msg, _x.right_msg, _x.pump_a, _x.pump_b,) = _get_struct_2I4f().unpack(str[start:end])
      self.custom_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I4f = None
def _get_struct_2I4f():
    global _struct_2I4f
    if _struct_2I4f is None:
        _struct_2I4f = struct.Struct("<2I4f")
    return _struct_2I4f
