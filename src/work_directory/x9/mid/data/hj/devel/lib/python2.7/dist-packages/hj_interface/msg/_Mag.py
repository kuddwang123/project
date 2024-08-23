# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/Mag.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class Mag(genpy.Message):
  _md5sum = "5cab63a10c5d367061e41cd1b7497522"
  _type = "hj_interface/Mag"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time custom_time
int16 mag_x
int16 mag_y
int16 mag_z
"""
  __slots__ = ['custom_time','mag_x','mag_y','mag_z']
  _slot_types = ['time','int16','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       custom_time,mag_x,mag_y,mag_z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Mag, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.custom_time is None:
        self.custom_time = genpy.Time()
      if self.mag_x is None:
        self.mag_x = 0
      if self.mag_y is None:
        self.mag_y = 0
      if self.mag_z is None:
        self.mag_z = 0
    else:
      self.custom_time = genpy.Time()
      self.mag_x = 0
      self.mag_y = 0
      self.mag_z = 0

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
      buff.write(_get_struct_2I3h().pack(_x.custom_time.secs, _x.custom_time.nsecs, _x.mag_x, _x.mag_y, _x.mag_z))
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
      end += 14
      (_x.custom_time.secs, _x.custom_time.nsecs, _x.mag_x, _x.mag_y, _x.mag_z,) = _get_struct_2I3h().unpack(str[start:end])
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
      buff.write(_get_struct_2I3h().pack(_x.custom_time.secs, _x.custom_time.nsecs, _x.mag_x, _x.mag_y, _x.mag_z))
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
      end += 14
      (_x.custom_time.secs, _x.custom_time.nsecs, _x.mag_x, _x.mag_y, _x.mag_z,) = _get_struct_2I3h().unpack(str[start:end])
      self.custom_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I3h = None
def _get_struct_2I3h():
    global _struct_2I3h
    if _struct_2I3h is None:
        _struct_2I3h = struct.Struct("<2I3h")
    return _struct_2I3h
