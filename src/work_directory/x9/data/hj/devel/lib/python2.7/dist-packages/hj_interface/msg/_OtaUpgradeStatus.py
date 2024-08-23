# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/OtaUpgradeStatus.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class OtaUpgradeStatus(genpy.Message):
  _md5sum = "7cb755155965318316b0e63093364936"
  _type = "hj_interface/OtaUpgradeStatus"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string   todo      # 当前操作为OTA升级 UpdateOTA：ota升级
int32    stage     # 0 下载 1 升级
float32  progress  # 下载或者升级进度
int32    module    # 1 mcu 2 power 3 core 注: stage=1时 关注此字段
int32    ret       # 升级结果 0 成功；-1 失败
string   msg       # 升级失败时，填充失败原因"""
  __slots__ = ['todo','stage','progress','module','ret','msg']
  _slot_types = ['string','int32','float32','int32','int32','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       todo,stage,progress,module,ret,msg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OtaUpgradeStatus, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.todo is None:
        self.todo = ''
      if self.stage is None:
        self.stage = 0
      if self.progress is None:
        self.progress = 0.
      if self.module is None:
        self.module = 0
      if self.ret is None:
        self.ret = 0
      if self.msg is None:
        self.msg = ''
    else:
      self.todo = ''
      self.stage = 0
      self.progress = 0.
      self.module = 0
      self.ret = 0
      self.msg = ''

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
      buff.write(_get_struct_if2i().pack(_x.stage, _x.progress, _x.module, _x.ret))
      _x = self.msg
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
      end += 16
      (_x.stage, _x.progress, _x.module, _x.ret,) = _get_struct_if2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msg = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.msg = str[start:end]
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
      buff.write(_get_struct_if2i().pack(_x.stage, _x.progress, _x.module, _x.ret))
      _x = self.msg
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
      end += 16
      (_x.stage, _x.progress, _x.module, _x.ret,) = _get_struct_if2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.msg = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.msg = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_if2i = None
def _get_struct_if2i():
    global _struct_if2i
    if _struct_if2i is None:
        _struct_if2i = struct.Struct("<if2i")
    return _struct_if2i
