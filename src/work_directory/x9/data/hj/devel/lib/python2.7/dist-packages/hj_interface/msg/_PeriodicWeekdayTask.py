# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/PeriodicWeekdayTask.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hj_interface.msg

class PeriodicWeekdayTask(genpy.Message):
  _md5sum = "b0391a22bc7c81642619e2d7209e30f3"
  _type = "hj_interface/PeriodicWeekdayTask"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string name_id
int32 weekday
string start_time
int32 smart        # 0-普通模式 1-智能模式
CleanAreas[] clean_areas
int32 clean_mode   # 1-变频 2-标准 3-深度
================================================================================
MSG: hj_interface/CleanAreas
int32 clean_area  #清扫区域 1-水面 2-池底 3-池壁
int32 count       #清扫次数
int32 time        #清扫时间单位分钟"""
  __slots__ = ['name_id','weekday','start_time','smart','clean_areas','clean_mode']
  _slot_types = ['string','int32','string','int32','hj_interface/CleanAreas[]','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name_id,weekday,start_time,smart,clean_areas,clean_mode

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PeriodicWeekdayTask, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.name_id is None:
        self.name_id = ''
      if self.weekday is None:
        self.weekday = 0
      if self.start_time is None:
        self.start_time = ''
      if self.smart is None:
        self.smart = 0
      if self.clean_areas is None:
        self.clean_areas = []
      if self.clean_mode is None:
        self.clean_mode = 0
    else:
      self.name_id = ''
      self.weekday = 0
      self.start_time = ''
      self.smart = 0
      self.clean_areas = []
      self.clean_mode = 0

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
      _x = self.name_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.weekday
      buff.write(_get_struct_i().pack(_x))
      _x = self.start_time
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.smart
      buff.write(_get_struct_i().pack(_x))
      length = len(self.clean_areas)
      buff.write(_struct_I.pack(length))
      for val1 in self.clean_areas:
        _x = val1
        buff.write(_get_struct_3i().pack(_x.clean_area, _x.count, _x.time))
      _x = self.clean_mode
      buff.write(_get_struct_i().pack(_x))
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
      if self.clean_areas is None:
        self.clean_areas = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.name_id = str[start:end]
      start = end
      end += 4
      (self.weekday,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.start_time = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.start_time = str[start:end]
      start = end
      end += 4
      (self.smart,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.clean_areas = []
      for i in range(0, length):
        val1 = hj_interface.msg.CleanAreas()
        _x = val1
        start = end
        end += 12
        (_x.clean_area, _x.count, _x.time,) = _get_struct_3i().unpack(str[start:end])
        self.clean_areas.append(val1)
      start = end
      end += 4
      (self.clean_mode,) = _get_struct_i().unpack(str[start:end])
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
      _x = self.name_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.weekday
      buff.write(_get_struct_i().pack(_x))
      _x = self.start_time
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.smart
      buff.write(_get_struct_i().pack(_x))
      length = len(self.clean_areas)
      buff.write(_struct_I.pack(length))
      for val1 in self.clean_areas:
        _x = val1
        buff.write(_get_struct_3i().pack(_x.clean_area, _x.count, _x.time))
      _x = self.clean_mode
      buff.write(_get_struct_i().pack(_x))
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
      if self.clean_areas is None:
        self.clean_areas = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.name_id = str[start:end]
      start = end
      end += 4
      (self.weekday,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.start_time = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.start_time = str[start:end]
      start = end
      end += 4
      (self.smart,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.clean_areas = []
      for i in range(0, length):
        val1 = hj_interface.msg.CleanAreas()
        _x = val1
        start = end
        end += 12
        (_x.clean_area, _x.count, _x.time,) = _get_struct_3i().unpack(str[start:end])
        self.clean_areas.append(val1)
      start = end
      end += 4
      (self.clean_mode,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3i = None
def _get_struct_3i():
    global _struct_3i
    if _struct_3i is None:
        _struct_3i = struct.Struct("<3i")
    return _struct_3i
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
