# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hj_interface/PeriodicTaskPkg.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import hj_interface.msg

class PeriodicTaskPkg(genpy.Message):
  _md5sum = "bdc52969980ff4c2018d392e4984338f"
  _type = "hj_interface/PeriodicTaskPkg"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 task_type         #0-快速任务 1-计时任务 2-周期任务 3-手动任务
int32 task_mode         #1-间隔天数周期任务 2-按周周期任务
int32 cancelNowTask     #是否取消今天的周期的任务
int32 taskExecute       #表示周期任务是否生效
PeriodicWeekdayTask[] weekday_task
PeriodicIntervalTask interval_task
================================================================================
MSG: hj_interface/PeriodicWeekdayTask
string name_id
int32 weekday
string start_time
int32 smart        # 0-普通模式 1-智能模式
CleanAreas[] clean_areas
int32 clean_mode   # 1-变频 2-标准 3-深度
================================================================================
MSG: hj_interface/CleanAreas
int32 clean_area  #清扫区域 1-水面 2-池底 3-池壁
int32 count       #清扫次数
int32 time        #清扫时间单位分钟
================================================================================
MSG: hj_interface/PeriodicIntervalTask
string name_id
string date
string start_time
int32 smart        # 0-普通模式 1-智能模式
int32 interval_days
CleanAreas[] clean_areas
int32 clean_mode   # 1-变频 2-标准 3-深度"""
  __slots__ = ['task_type','task_mode','cancelNowTask','taskExecute','weekday_task','interval_task']
  _slot_types = ['int32','int32','int32','int32','hj_interface/PeriodicWeekdayTask[]','hj_interface/PeriodicIntervalTask']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       task_type,task_mode,cancelNowTask,taskExecute,weekday_task,interval_task

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PeriodicTaskPkg, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.task_type is None:
        self.task_type = 0
      if self.task_mode is None:
        self.task_mode = 0
      if self.cancelNowTask is None:
        self.cancelNowTask = 0
      if self.taskExecute is None:
        self.taskExecute = 0
      if self.weekday_task is None:
        self.weekday_task = []
      if self.interval_task is None:
        self.interval_task = hj_interface.msg.PeriodicIntervalTask()
    else:
      self.task_type = 0
      self.task_mode = 0
      self.cancelNowTask = 0
      self.taskExecute = 0
      self.weekday_task = []
      self.interval_task = hj_interface.msg.PeriodicIntervalTask()

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
      buff.write(_get_struct_4i().pack(_x.task_type, _x.task_mode, _x.cancelNowTask, _x.taskExecute))
      length = len(self.weekday_task)
      buff.write(_struct_I.pack(length))
      for val1 in self.weekday_task:
        _x = val1.name_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.weekday
        buff.write(_get_struct_i().pack(_x))
        _x = val1.start_time
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.smart
        buff.write(_get_struct_i().pack(_x))
        length = len(val1.clean_areas)
        buff.write(_struct_I.pack(length))
        for val2 in val1.clean_areas:
          _x = val2
          buff.write(_get_struct_3i().pack(_x.clean_area, _x.count, _x.time))
        _x = val1.clean_mode
        buff.write(_get_struct_i().pack(_x))
      _x = self.interval_task.name_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.interval_task.date
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.interval_task.start_time
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2i().pack(_x.interval_task.smart, _x.interval_task.interval_days))
      length = len(self.interval_task.clean_areas)
      buff.write(_struct_I.pack(length))
      for val1 in self.interval_task.clean_areas:
        _x = val1
        buff.write(_get_struct_3i().pack(_x.clean_area, _x.count, _x.time))
      _x = self.interval_task.clean_mode
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
      if self.weekday_task is None:
        self.weekday_task = None
      if self.interval_task is None:
        self.interval_task = hj_interface.msg.PeriodicIntervalTask()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.task_type, _x.task_mode, _x.cancelNowTask, _x.taskExecute,) = _get_struct_4i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.weekday_task = []
      for i in range(0, length):
        val1 = hj_interface.msg.PeriodicWeekdayTask()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name_id = str[start:end]
        start = end
        end += 4
        (val1.weekday,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.start_time = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.start_time = str[start:end]
        start = end
        end += 4
        (val1.smart,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.clean_areas = []
        for i in range(0, length):
          val2 = hj_interface.msg.CleanAreas()
          _x = val2
          start = end
          end += 12
          (_x.clean_area, _x.count, _x.time,) = _get_struct_3i().unpack(str[start:end])
          val1.clean_areas.append(val2)
        start = end
        end += 4
        (val1.clean_mode,) = _get_struct_i().unpack(str[start:end])
        self.weekday_task.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interval_task.name_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.interval_task.name_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interval_task.date = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.interval_task.date = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interval_task.start_time = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.interval_task.start_time = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.interval_task.smart, _x.interval_task.interval_days,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.interval_task.clean_areas = []
      for i in range(0, length):
        val1 = hj_interface.msg.CleanAreas()
        _x = val1
        start = end
        end += 12
        (_x.clean_area, _x.count, _x.time,) = _get_struct_3i().unpack(str[start:end])
        self.interval_task.clean_areas.append(val1)
      start = end
      end += 4
      (self.interval_task.clean_mode,) = _get_struct_i().unpack(str[start:end])
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
      buff.write(_get_struct_4i().pack(_x.task_type, _x.task_mode, _x.cancelNowTask, _x.taskExecute))
      length = len(self.weekday_task)
      buff.write(_struct_I.pack(length))
      for val1 in self.weekday_task:
        _x = val1.name_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.weekday
        buff.write(_get_struct_i().pack(_x))
        _x = val1.start_time
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.smart
        buff.write(_get_struct_i().pack(_x))
        length = len(val1.clean_areas)
        buff.write(_struct_I.pack(length))
        for val2 in val1.clean_areas:
          _x = val2
          buff.write(_get_struct_3i().pack(_x.clean_area, _x.count, _x.time))
        _x = val1.clean_mode
        buff.write(_get_struct_i().pack(_x))
      _x = self.interval_task.name_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.interval_task.date
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.interval_task.start_time
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2i().pack(_x.interval_task.smart, _x.interval_task.interval_days))
      length = len(self.interval_task.clean_areas)
      buff.write(_struct_I.pack(length))
      for val1 in self.interval_task.clean_areas:
        _x = val1
        buff.write(_get_struct_3i().pack(_x.clean_area, _x.count, _x.time))
      _x = self.interval_task.clean_mode
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
      if self.weekday_task is None:
        self.weekday_task = None
      if self.interval_task is None:
        self.interval_task = hj_interface.msg.PeriodicIntervalTask()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.task_type, _x.task_mode, _x.cancelNowTask, _x.taskExecute,) = _get_struct_4i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.weekday_task = []
      for i in range(0, length):
        val1 = hj_interface.msg.PeriodicWeekdayTask()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name_id = str[start:end]
        start = end
        end += 4
        (val1.weekday,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.start_time = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.start_time = str[start:end]
        start = end
        end += 4
        (val1.smart,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.clean_areas = []
        for i in range(0, length):
          val2 = hj_interface.msg.CleanAreas()
          _x = val2
          start = end
          end += 12
          (_x.clean_area, _x.count, _x.time,) = _get_struct_3i().unpack(str[start:end])
          val1.clean_areas.append(val2)
        start = end
        end += 4
        (val1.clean_mode,) = _get_struct_i().unpack(str[start:end])
        self.weekday_task.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interval_task.name_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.interval_task.name_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interval_task.date = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.interval_task.date = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.interval_task.start_time = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.interval_task.start_time = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.interval_task.smart, _x.interval_task.interval_days,) = _get_struct_2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.interval_task.clean_areas = []
      for i in range(0, length):
        val1 = hj_interface.msg.CleanAreas()
        _x = val1
        start = end
        end += 12
        (_x.clean_area, _x.count, _x.time,) = _get_struct_3i().unpack(str[start:end])
        self.interval_task.clean_areas.append(val1)
      start = end
      end += 4
      (self.interval_task.clean_mode,) = _get_struct_i().unpack(str[start:end])
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
_struct_3i = None
def _get_struct_3i():
    global _struct_3i
    if _struct_3i is None:
        _struct_3i = struct.Struct("<3i")
    return _struct_3i
_struct_4i = None
def _get_struct_4i():
    global _struct_4i
    if _struct_4i is None:
        _struct_4i = struct.Struct("<4i")
    return _struct_4i
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
