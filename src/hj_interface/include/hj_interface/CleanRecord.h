// Generated by gencpp from file hj_interface/CleanRecord.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_CLEANRECORD_H
#define HJ_INTERFACE_MESSAGE_CLEANRECORD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hj_interface
{
template <class ContainerAllocator>
struct CleanRecord_
{
  typedef CleanRecord_<ContainerAllocator> Type;

  CleanRecord_()
    : clean_speed(0.0)
    , surface_clean_area(0.0)
    , bottom_clean_area(0.0)
    , wall_clean_area(0.0)
    , water_line_clean_area(0.0)
    , pool_area(0.0)
    , pool_volume(0.0)
    , pool_depth(0.0)
    , pool_little_depth(0.0)
    , pool_shape(0)
    , map_line_file_path()
    , avoidObstaclesCount(0)
    , getOutCount(0)
    , multiple_bottom(0)
    , clean_area(0.0)
    , trapped_count(0)
    , edge_time(0)
    , fall_count(0)
    , avoid_fall(0)
    , clean_time()
    , deepth_list()  {
    }
  CleanRecord_(const ContainerAllocator& _alloc)
    : clean_speed(0.0)
    , surface_clean_area(0.0)
    , bottom_clean_area(0.0)
    , wall_clean_area(0.0)
    , water_line_clean_area(0.0)
    , pool_area(0.0)
    , pool_volume(0.0)
    , pool_depth(0.0)
    , pool_little_depth(0.0)
    , pool_shape(0)
    , map_line_file_path(_alloc)
    , avoidObstaclesCount(0)
    , getOutCount(0)
    , multiple_bottom(0)
    , clean_area(0.0)
    , trapped_count(0)
    , edge_time(0)
    , fall_count(0)
    , avoid_fall(0)
    , clean_time(_alloc)
    , deepth_list(_alloc)  {
  (void)_alloc;
    }



   typedef float _clean_speed_type;
  _clean_speed_type clean_speed;

   typedef float _surface_clean_area_type;
  _surface_clean_area_type surface_clean_area;

   typedef float _bottom_clean_area_type;
  _bottom_clean_area_type bottom_clean_area;

   typedef float _wall_clean_area_type;
  _wall_clean_area_type wall_clean_area;

   typedef float _water_line_clean_area_type;
  _water_line_clean_area_type water_line_clean_area;

   typedef float _pool_area_type;
  _pool_area_type pool_area;

   typedef float _pool_volume_type;
  _pool_volume_type pool_volume;

   typedef float _pool_depth_type;
  _pool_depth_type pool_depth;

   typedef float _pool_little_depth_type;
  _pool_little_depth_type pool_little_depth;

   typedef int32_t _pool_shape_type;
  _pool_shape_type pool_shape;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _map_line_file_path_type;
  _map_line_file_path_type map_line_file_path;

   typedef int32_t _avoidObstaclesCount_type;
  _avoidObstaclesCount_type avoidObstaclesCount;

   typedef int32_t _getOutCount_type;
  _getOutCount_type getOutCount;

   typedef int32_t _multiple_bottom_type;
  _multiple_bottom_type multiple_bottom;

   typedef float _clean_area_type;
  _clean_area_type clean_area;

   typedef int32_t _trapped_count_type;
  _trapped_count_type trapped_count;

   typedef int32_t _edge_time_type;
  _edge_time_type edge_time;

   typedef int32_t _fall_count_type;
  _fall_count_type fall_count;

   typedef int32_t _avoid_fall_type;
  _avoid_fall_type avoid_fall;

   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _clean_time_type;
  _clean_time_type clean_time;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _deepth_list_type;
  _deepth_list_type deepth_list;





  typedef boost::shared_ptr< ::hj_interface::CleanRecord_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::CleanRecord_<ContainerAllocator> const> ConstPtr;

}; // struct CleanRecord_

typedef ::hj_interface::CleanRecord_<std::allocator<void> > CleanRecord;

typedef boost::shared_ptr< ::hj_interface::CleanRecord > CleanRecordPtr;
typedef boost::shared_ptr< ::hj_interface::CleanRecord const> CleanRecordConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::CleanRecord_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::CleanRecord_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::CleanRecord_<ContainerAllocator1> & lhs, const ::hj_interface::CleanRecord_<ContainerAllocator2> & rhs)
{
  return lhs.clean_speed == rhs.clean_speed &&
    lhs.surface_clean_area == rhs.surface_clean_area &&
    lhs.bottom_clean_area == rhs.bottom_clean_area &&
    lhs.wall_clean_area == rhs.wall_clean_area &&
    lhs.water_line_clean_area == rhs.water_line_clean_area &&
    lhs.pool_area == rhs.pool_area &&
    lhs.pool_volume == rhs.pool_volume &&
    lhs.pool_depth == rhs.pool_depth &&
    lhs.pool_little_depth == rhs.pool_little_depth &&
    lhs.pool_shape == rhs.pool_shape &&
    lhs.map_line_file_path == rhs.map_line_file_path &&
    lhs.avoidObstaclesCount == rhs.avoidObstaclesCount &&
    lhs.getOutCount == rhs.getOutCount &&
    lhs.multiple_bottom == rhs.multiple_bottom &&
    lhs.clean_area == rhs.clean_area &&
    lhs.trapped_count == rhs.trapped_count &&
    lhs.edge_time == rhs.edge_time &&
    lhs.fall_count == rhs.fall_count &&
    lhs.avoid_fall == rhs.avoid_fall &&
    lhs.clean_time == rhs.clean_time &&
    lhs.deepth_list == rhs.deepth_list;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::CleanRecord_<ContainerAllocator1> & lhs, const ::hj_interface::CleanRecord_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::CleanRecord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::CleanRecord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::CleanRecord_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::CleanRecord_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::CleanRecord_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::CleanRecord_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::CleanRecord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "95e5b9a2ae22cdd8e2a516d3ada5579e";
  }

  static const char* value(const ::hj_interface::CleanRecord_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x95e5b9a2ae22cdd8ULL;
  static const uint64_t static_value2 = 0xe2a516d3ada5579eULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::CleanRecord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/CleanRecord";
  }

  static const char* value(const ::hj_interface::CleanRecord_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::CleanRecord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## 清洁日志消息\n"
"float32 clean_speed             # 清洁速度 单位：m/s\n"
"float32 surface_clean_area      # 清洁水面面积 单位：m2\n"
"float32 bottom_clean_area       # 清洁池底面积\n"
"float32 wall_clean_area         # 清洁池壁面积\n"
"float32 water_line_clean_area   # 清洁水线面积\n"
"float32 pool_area               # 泳池面积\n"
"float32 pool_volume             # 泳池体积\n"
"float32 pool_depth              # 泳池最大深度\n"
"float32 pool_little_depth       # 泳池最小深度\n"
"int32   pool_shape              # 泳池形状\n"
"string  map_line_file_path      # 地图轨迹文件路径\n"
"int32   avoidObstaclesCount     # 避障次数\n"
"int32   getOutCount             # 脱困次数\n"
"int32   multiple_bottom         # 多层池底 1-单层 2-多层\n"
"\n"
"#### 以下用于埋点的消息\n"
"## 清洁事件消息- 与清洁日志消息高度重叠\n"
"\n"
"## 清洁不同区域共同信息\n"
"float32 clean_area             # 清扫面积\n"
"int32 trapped_count            # 被困次数\n"
"\n"
"## 水面清洁信息\n"
"int32 edge_time                # 延边时长\n"
"\n"
"## 池底清洁信息 - 无多余信息\n"
"\n"
"## 池壁清洁信息 - 无多余信息\n"
"\n"
"## 水线清洁信息 - 无多余信息\n"
"\n"
"## 多平台清洁信息\n"
"int32 fall_count               # 跌落次数\n"
"int32 avoid_fall               # 防跌落次数\n"
"int32[] clean_time               # 清洁时长\n"
"float32[] deepth_list          # 深度列表\n"
;
  }

  static const char* value(const ::hj_interface::CleanRecord_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::CleanRecord_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.clean_speed);
      stream.next(m.surface_clean_area);
      stream.next(m.bottom_clean_area);
      stream.next(m.wall_clean_area);
      stream.next(m.water_line_clean_area);
      stream.next(m.pool_area);
      stream.next(m.pool_volume);
      stream.next(m.pool_depth);
      stream.next(m.pool_little_depth);
      stream.next(m.pool_shape);
      stream.next(m.map_line_file_path);
      stream.next(m.avoidObstaclesCount);
      stream.next(m.getOutCount);
      stream.next(m.multiple_bottom);
      stream.next(m.clean_area);
      stream.next(m.trapped_count);
      stream.next(m.edge_time);
      stream.next(m.fall_count);
      stream.next(m.avoid_fall);
      stream.next(m.clean_time);
      stream.next(m.deepth_list);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CleanRecord_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::CleanRecord_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::CleanRecord_<ContainerAllocator>& v)
  {
    s << indent << "clean_speed: ";
    Printer<float>::stream(s, indent + "  ", v.clean_speed);
    s << indent << "surface_clean_area: ";
    Printer<float>::stream(s, indent + "  ", v.surface_clean_area);
    s << indent << "bottom_clean_area: ";
    Printer<float>::stream(s, indent + "  ", v.bottom_clean_area);
    s << indent << "wall_clean_area: ";
    Printer<float>::stream(s, indent + "  ", v.wall_clean_area);
    s << indent << "water_line_clean_area: ";
    Printer<float>::stream(s, indent + "  ", v.water_line_clean_area);
    s << indent << "pool_area: ";
    Printer<float>::stream(s, indent + "  ", v.pool_area);
    s << indent << "pool_volume: ";
    Printer<float>::stream(s, indent + "  ", v.pool_volume);
    s << indent << "pool_depth: ";
    Printer<float>::stream(s, indent + "  ", v.pool_depth);
    s << indent << "pool_little_depth: ";
    Printer<float>::stream(s, indent + "  ", v.pool_little_depth);
    s << indent << "pool_shape: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pool_shape);
    s << indent << "map_line_file_path: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.map_line_file_path);
    s << indent << "avoidObstaclesCount: ";
    Printer<int32_t>::stream(s, indent + "  ", v.avoidObstaclesCount);
    s << indent << "getOutCount: ";
    Printer<int32_t>::stream(s, indent + "  ", v.getOutCount);
    s << indent << "multiple_bottom: ";
    Printer<int32_t>::stream(s, indent + "  ", v.multiple_bottom);
    s << indent << "clean_area: ";
    Printer<float>::stream(s, indent + "  ", v.clean_area);
    s << indent << "trapped_count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.trapped_count);
    s << indent << "edge_time: ";
    Printer<int32_t>::stream(s, indent + "  ", v.edge_time);
    s << indent << "fall_count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.fall_count);
    s << indent << "avoid_fall: ";
    Printer<int32_t>::stream(s, indent + "  ", v.avoid_fall);
    s << indent << "clean_time[]" << std::endl;
    for (size_t i = 0; i < v.clean_time.size(); ++i)
    {
      s << indent << "  clean_time[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.clean_time[i]);
    }
    s << indent << "deepth_list[]" << std::endl;
    for (size_t i = 0; i < v.deepth_list.size(); ++i)
    {
      s << indent << "  deepth_list[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.deepth_list[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_CLEANRECORD_H
