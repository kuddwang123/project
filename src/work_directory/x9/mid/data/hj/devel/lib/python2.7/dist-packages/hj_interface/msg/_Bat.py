# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/Bat.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Bat(genpy.Message):
  _md5sum = "767adf0ab14ff9bd8605abb0c5fb3f5b"
  _type = "hj_interface/Bat"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint16 vol
uint16 ch_vol
int16 cur
uint8 power
int16 temperature
"""
  __slots__ = ['vol','ch_vol','cur','power','temperature']
  _slot_types = ['uint16','uint16','int16','uint8','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       vol,ch_vol,cur,power,temperature

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Bat, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.vol is None:
        self.vol = 0
      if self.ch_vol is None:
        self.ch_vol = 0
      if self.cur is None:
        self.cur = 0
      if self.power is None:
        self.power = 0
      if self.temperature is None:
        self.temperature = 0
    else:
      self.vol = 0
      self.ch_vol = 0
      self.cur = 0
      self.power = 0
      self.temperature = 0

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
      buff.write(_get_struct_2HhBh().pack(_x.vol, _x.ch_vol, _x.cur, _x.power, _x.temperature))
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
      end += 9
      (_x.vol, _x.ch_vol, _x.cur, _x.power, _x.temperature,) = _get_struct_2HhBh().unpack(str[start:end])
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
      buff.write(_get_struct_2HhBh().pack(_x.vol, _x.ch_vol, _x.cur, _x.power, _x.temperature))
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
      end += 9
      (_x.vol, _x.ch_vol, _x.cur, _x.power, _x.temperature,) = _get_struct_2HhBh().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2HhBh = None
def _get_struct_2HhBh():
    global _struct_2HhBh
    if _struct_2HhBh is None:
        _struct_2HhBh = struct.Struct("<2HhBh")
    return _struct_2HhBh
