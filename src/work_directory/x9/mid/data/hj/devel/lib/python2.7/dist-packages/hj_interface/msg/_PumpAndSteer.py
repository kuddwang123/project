# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/PumpAndSteer.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class PumpAndSteer(genpy.Message):
  _md5sum = "d097c874865c9dec50e5a9f2ca806621"
  _type = "hj_interface/PumpAndSteer"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time timestamp
uint8 turn_motor_ctl     #!< 0 刹车； 1 运动； 2 滑行 
int16 turn_motor        #!< 矢量碰口电机， 0，90，180>

uint8 pump_ctl       #!< 0 停止； 1 运行 水泵控制          
uint8 pump_speed_l       #!< 占空比 0到100 水泵PWM控制 左边水泵
uint8 pump_speed_r      #!< 占空比 0到100 水泵PWM控制 右边水泵
"""
  __slots__ = ['timestamp','turn_motor_ctl','turn_motor','pump_ctl','pump_speed_l','pump_speed_r']
  _slot_types = ['time','uint8','int16','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,turn_motor_ctl,turn_motor,pump_ctl,pump_speed_l,pump_speed_r

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PumpAndSteer, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.turn_motor_ctl is None:
        self.turn_motor_ctl = 0
      if self.turn_motor is None:
        self.turn_motor = 0
      if self.pump_ctl is None:
        self.pump_ctl = 0
      if self.pump_speed_l is None:
        self.pump_speed_l = 0
      if self.pump_speed_r is None:
        self.pump_speed_r = 0
    else:
      self.timestamp = genpy.Time()
      self.turn_motor_ctl = 0
      self.turn_motor = 0
      self.pump_ctl = 0
      self.pump_speed_l = 0
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
      buff.write(_get_struct_2IBh3B().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.turn_motor_ctl, _x.turn_motor, _x.pump_ctl, _x.pump_speed_l, _x.pump_speed_r))
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
      end += 14
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.turn_motor_ctl, _x.turn_motor, _x.pump_ctl, _x.pump_speed_l, _x.pump_speed_r,) = _get_struct_2IBh3B().unpack(str[start:end])
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
      buff.write(_get_struct_2IBh3B().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.turn_motor_ctl, _x.turn_motor, _x.pump_ctl, _x.pump_speed_l, _x.pump_speed_r))
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
      end += 14
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.turn_motor_ctl, _x.turn_motor, _x.pump_ctl, _x.pump_speed_l, _x.pump_speed_r,) = _get_struct_2IBh3B().unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2IBh3B = None
def _get_struct_2IBh3B():
    global _struct_2IBh3B
    if _struct_2IBh3B is None:
        _struct_2IBh3B = struct.Struct("<2IBh3B")
    return _struct_2IBh3B
