# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/Bat.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Bat(genpy.Message):
  _md5sum = "67beab113d23636ee0599f8ce7cdb781"
  _type = "hj_interface/Bat"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8   power              #电量%
int8    temp1              #温度1 
int8    temp2              #温度2
int8    temp3              #温度3
uint16  bat_vol            #电池电压mV
int16   bat_disch_cur      #电池放电电流mA
uint16  ch_vol             #电池充电电压mV
int16   charger_ch_cur     #充电端充电电流mA
uint16  bat_cycle_times    #电池循环次数
uint8   bat_health_left    #电池健康剩余容量
"""
  __slots__ = ['power','temp1','temp2','temp3','bat_vol','bat_disch_cur','ch_vol','charger_ch_cur','bat_cycle_times','bat_health_left']
  _slot_types = ['uint8','int8','int8','int8','uint16','int16','uint16','int16','uint16','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       power,temp1,temp2,temp3,bat_vol,bat_disch_cur,ch_vol,charger_ch_cur,bat_cycle_times,bat_health_left

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Bat, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.power is None:
        self.power = 0
      if self.temp1 is None:
        self.temp1 = 0
      if self.temp2 is None:
        self.temp2 = 0
      if self.temp3 is None:
        self.temp3 = 0
      if self.bat_vol is None:
        self.bat_vol = 0
      if self.bat_disch_cur is None:
        self.bat_disch_cur = 0
      if self.ch_vol is None:
        self.ch_vol = 0
      if self.charger_ch_cur is None:
        self.charger_ch_cur = 0
      if self.bat_cycle_times is None:
        self.bat_cycle_times = 0
      if self.bat_health_left is None:
        self.bat_health_left = 0
    else:
      self.power = 0
      self.temp1 = 0
      self.temp2 = 0
      self.temp3 = 0
      self.bat_vol = 0
      self.bat_disch_cur = 0
      self.ch_vol = 0
      self.charger_ch_cur = 0
      self.bat_cycle_times = 0
      self.bat_health_left = 0

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
      buff.write(_get_struct_B3bHhHhHB().pack(_x.power, _x.temp1, _x.temp2, _x.temp3, _x.bat_vol, _x.bat_disch_cur, _x.ch_vol, _x.charger_ch_cur, _x.bat_cycle_times, _x.bat_health_left))
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
      end = 0
      _x = self
      start = end
      end += 15
      (_x.power, _x.temp1, _x.temp2, _x.temp3, _x.bat_vol, _x.bat_disch_cur, _x.ch_vol, _x.charger_ch_cur, _x.bat_cycle_times, _x.bat_health_left,) = _get_struct_B3bHhHhHB().unpack(str[start:end])
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
      buff.write(_get_struct_B3bHhHhHB().pack(_x.power, _x.temp1, _x.temp2, _x.temp3, _x.bat_vol, _x.bat_disch_cur, _x.ch_vol, _x.charger_ch_cur, _x.bat_cycle_times, _x.bat_health_left))
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
      end = 0
      _x = self
      start = end
      end += 15
      (_x.power, _x.temp1, _x.temp2, _x.temp3, _x.bat_vol, _x.bat_disch_cur, _x.ch_vol, _x.charger_ch_cur, _x.bat_cycle_times, _x.bat_health_left,) = _get_struct_B3bHhHhHB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B3bHhHhHB = None
def _get_struct_B3bHhHhHB():
    global _struct_B3bHhHhHB
    if _struct_B3bHhHhHB is None:
        _struct_B3bHhHhHB = struct.Struct("<B3bHhHhHB")
    return _struct_B3bHhHhHB
