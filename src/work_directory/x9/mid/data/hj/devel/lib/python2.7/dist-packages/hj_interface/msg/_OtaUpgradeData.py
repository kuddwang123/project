# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/OtaUpgradeData.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hj_interface.msg

class OtaUpgradeData(genpy.Message):
  _md5sum = "cf34e244f7cadb161b4fabd080ab378c"
  _type = "hj_interface/OtaUpgradeData"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string      todo   # 当前操作为OTA升级 UpdateOTA: ota升级 Reboot: 机器重启
int32       stage  # 0 下载 1 升级
int32       module # 1 mcu 2 power 3 core 注: stage=1时 关注此字段
subPackData data   # 固件包子包的地址和md5
================================================================================
MSG: hj_interface/subPackData
string addr # 子固件包的存放位置
string ver  # 子固件包的版本号"""
  __slots__ = ['todo','stage','module','data']
  _slot_types = ['string','int32','int32','hj_interface/subPackData']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       todo,stage,module,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OtaUpgradeData, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.todo is None:
        self.todo = ''
      if self.stage is None:
        self.stage = 0
      if self.module is None:
        self.module = 0
      if self.data is None:
        self.data = hj_interface.msg.subPackData()
    else:
      self.todo = ''
      self.stage = 0
      self.module = 0
      self.data = hj_interface.msg.subPackData()

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
      _x = self.todo
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2i().pack(_x.stage, _x.module))
      _x = self.data.addr
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.data.ver
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
      if self.data is None:
        self.data = hj_interface.msg.subPackData()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.todo = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.todo = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.stage, _x.module,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.data.addr = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.data.addr = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.data.ver = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.data.ver = str[start:end]
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
      _x = self.todo
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2i().pack(_x.stage, _x.module))
      _x = self.data.addr
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.data.ver
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
      if self.data is None:
        self.data = hj_interface.msg.subPackData()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.todo = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.todo = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.stage, _x.module,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.data.addr = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.data.addr = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.data.ver = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.data.ver = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
