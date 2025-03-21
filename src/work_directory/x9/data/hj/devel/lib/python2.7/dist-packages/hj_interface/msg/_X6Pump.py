# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/X6Pump.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class X6Pump(genpy.Message):
  _md5sum = "5e2db79fc592cb10d575a4498590c4ac"
  _type = "hj_interface/X6Pump"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time timestamp

uint8 pump_ctl_l       #!< 0 停止； 1 运行 水泵控制   左侧       
uint8 pump_speed_l       #!< 占空比 0到100 水泵PWM控制   左侧

uint8 pump_ctl_r       #!< 0 停止； 1 运行 水泵控制  右侧        
uint8 pump_speed_r       #!< 占空比 0到100 水泵PWM控制  右侧"""
  __slots__ = ['timestamp','pump_ctl_l','pump_speed_l','pump_ctl_r','pump_speed_r']
  _slot_types = ['time','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,pump_ctl_l,pump_speed_l,pump_ctl_r,pump_speed_r

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(X6Pump, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.pump_ctl_l is None:
        self.pump_ctl_l = 0
      if self.pump_speed_l is None:
        self.pump_speed_l = 0
      if self.pump_ctl_r is None:
        self.pump_ctl_r = 0
      if self.pump_speed_r is None:
        self.pump_speed_r = 0
    else:
      self.timestamp = genpy.Time()
      self.pump_ctl_l = 0
      self.pump_speed_l = 0
      self.pump_ctl_r = 0
      self.pump_speed_r = 0

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
      buff.write(_get_struct_2I4B().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.pump_ctl_l, _x.pump_speed_l, _x.pump_ctl_r, _x.pump_speed_r))
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
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.pump_ctl_l, _x.pump_speed_l, _x.pump_ctl_r, _x.pump_speed_r,) = _get_struct_2I4B().unpack(str[start:end])
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
      buff.write(_get_struct_2I4B().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.pump_ctl_l, _x.pump_speed_l, _x.pump_ctl_r, _x.pump_speed_r))
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
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.pump_ctl_l, _x.pump_speed_l, _x.pump_ctl_r, _x.pump_speed_r,) = _get_struct_2I4B().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I4B = None
def _get_struct_2I4B():
    global _struct_2I4B
    if _struct_2I4B is None:
        _struct_2I4B = struct.Struct("<2I4B")
    return _struct_2I4B
