# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/SlamActionRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SlamActionRequest(genpy.Message):
  _md5sum = "4f5f59ca82f166afa80831a42f295d78"
  _type = "hj_interface/SlamActionRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 action_cmd    # 1: slam建图 2: slam重定位
                    # 11: 清扫水面 12: 池底清扫 13: 池壁清扫
                    # 18: 姿态调整
                    # 19: 清洁模式
                    # 21: 召回 22: 回充
                    # 31: slam状态请求 32: navi状态请求 35: 获取清扫记录
                    # 103: slam停止建图 104: slam停止定位
                    # 105：navi停止建图/重定位延边 106：navi停止极限延边
                    # 111: 暂停清扫 112: 继续清扫 113: 停止清扫
                    # 123: 停止召回 124: 停止回充
"""
  __slots__ = ['action_cmd']
  _slot_types = ['uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       action_cmd

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SlamActionRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.action_cmd is None:
        self.action_cmd = 0
    else:
      self.action_cmd = 0

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
      _x = self.action_cmd
      buff.write(_get_struct_B().pack(_x))
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
      start = end
      end += 1
      (self.action_cmd,) = _get_struct_B().unpack(str[start:end])
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
      _x = self.action_cmd
      buff.write(_get_struct_B().pack(_x))
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
      start = end
      end += 1
      (self.action_cmd,) = _get_struct_B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/SlamActionResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hj_interface.msg

class SlamActionResponse(genpy.Message):
  _md5sum = "dc13bf94fa3ab8d539d14adc72a924bd"
  _type = "hj_interface/SlamActionResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint8 result                # 0: 收到 
                            # 1: slam待机 2: slam建图中 3: slam定位中 
CleanRecord CleanRecord     # 清扫记录

================================================================================
MSG: hj_interface/CleanRecord
float32 clean_speed             # 清洁速度 单位：m/s
float32 surface_clean_area      # 清洁水面面积 单位：m2
float32 bottom_clean_area       # 清洁池底面积
float32 wall_clean_area         # 清洁池壁面积
float32 pool_area               # 泳池面积
float32 pool_volume             # 泳池体积
string map_line_file_path       # 地图轨迹文件路径

## not to be reported for the time being
#uint8 avoid_obstacles          # 避障次数
#int32 get_out_time             # 脱困时长 单位：秒
#uint8 get_out_failed           # 脱困失败次数
#uint8 get_out_success          # 脱困成功次数
#float32 supple_clean_area      # 补扫面积 """
  __slots__ = ['result','CleanRecord']
  _slot_types = ['uint8','hj_interface/CleanRecord']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result,CleanRecord

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SlamActionResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = 0
      if self.CleanRecord is None:
        self.CleanRecord = hj_interface.msg.CleanRecord()
    else:
      self.result = 0
      self.CleanRecord = hj_interface.msg.CleanRecord()

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
      buff.write(_get_struct_B6f().pack(_x.result, _x.CleanRecord.clean_speed, _x.CleanRecord.surface_clean_area, _x.CleanRecord.bottom_clean_area, _x.CleanRecord.wall_clean_area, _x.CleanRecord.pool_area, _x.CleanRecord.pool_volume))
      _x = self.CleanRecord.map_line_file_path
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.CleanRecord is None:
        self.CleanRecord = hj_interface.msg.CleanRecord()
      end = 0
      _x = self
      start = end
      end += 25
      (_x.result, _x.CleanRecord.clean_speed, _x.CleanRecord.surface_clean_area, _x.CleanRecord.bottom_clean_area, _x.CleanRecord.wall_clean_area, _x.CleanRecord.pool_area, _x.CleanRecord.pool_volume,) = _get_struct_B6f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.CleanRecord.map_line_file_path = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.CleanRecord.map_line_file_path = str[start:end]
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
      buff.write(_get_struct_B6f().pack(_x.result, _x.CleanRecord.clean_speed, _x.CleanRecord.surface_clean_area, _x.CleanRecord.bottom_clean_area, _x.CleanRecord.wall_clean_area, _x.CleanRecord.pool_area, _x.CleanRecord.pool_volume))
      _x = self.CleanRecord.map_line_file_path
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.CleanRecord is None:
        self.CleanRecord = hj_interface.msg.CleanRecord()
      end = 0
      _x = self
      start = end
      end += 25
      (_x.result, _x.CleanRecord.clean_speed, _x.CleanRecord.surface_clean_area, _x.CleanRecord.bottom_clean_area, _x.CleanRecord.wall_clean_area, _x.CleanRecord.pool_area, _x.CleanRecord.pool_volume,) = _get_struct_B6f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.CleanRecord.map_line_file_path = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.CleanRecord.map_line_file_path = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B6f = None
def _get_struct_B6f():
    global _struct_B6f
    if _struct_B6f is None:
        _struct_B6f = struct.Struct("<B6f")
    return _struct_B6f
class SlamAction(object):
  _type          = 'hj_interface/SlamAction'
  _md5sum = '32cfa97b583cff271f1a21d44578aac5'
  _request_class  = SlamActionRequest
  _response_class = SlamActionResponse
