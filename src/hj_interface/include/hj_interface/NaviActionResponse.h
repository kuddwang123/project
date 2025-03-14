// Generated by gencpp from file hj_interface/NaviActionResponse.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_NAVIACTIONRESPONSE_H
#define HJ_INTERFACE_MESSAGE_NAVIACTIONRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hj_interface/CleanRecord.h>

namespace hj_interface
{
template <class ContainerAllocator>
struct NaviActionResponse_
{
  typedef NaviActionResponse_<ContainerAllocator> Type;

  NaviActionResponse_()
    : result(0)
    , CleanRecord()  {
    }
  NaviActionResponse_(const ContainerAllocator& _alloc)
    : result(0)
    , CleanRecord(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _result_type;
  _result_type result;

   typedef  ::hj_interface::CleanRecord_<ContainerAllocator>  _CleanRecord_type;
  _CleanRecord_type CleanRecord;





  typedef boost::shared_ptr< ::hj_interface::NaviActionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::NaviActionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct NaviActionResponse_

typedef ::hj_interface::NaviActionResponse_<std::allocator<void> > NaviActionResponse;

typedef boost::shared_ptr< ::hj_interface::NaviActionResponse > NaviActionResponsePtr;
typedef boost::shared_ptr< ::hj_interface::NaviActionResponse const> NaviActionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::NaviActionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::NaviActionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::NaviActionResponse_<ContainerAllocator1> & lhs, const ::hj_interface::NaviActionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result &&
    lhs.CleanRecord == rhs.CleanRecord;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::NaviActionResponse_<ContainerAllocator1> & lhs, const ::hj_interface::NaviActionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::NaviActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::NaviActionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::NaviActionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e275c9e697e564418602cf0f0c89a649";
  }

  static const char* value(const ::hj_interface::NaviActionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe275c9e697e56441ULL;
  static const uint64_t static_value2 = 0x8602cf0f0c89a649ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/NaviActionResponse";
  }

  static const char* value(const ::hj_interface::NaviActionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint8 result                # 0: 收到 \n"
"                            # 11: navi待机 12: navi清扫中\n"
"                            # 16：navi建图/重定位延边中 17：navi极限延边中\n"
"                            # 18：navi姿态调整中\n"
"                            # 21: navi召回中 22: navi回充中\n"
"CleanRecord CleanRecord     # 清扫记录\n"
"\n"
"================================================================================\n"
"MSG: hj_interface/CleanRecord\n"
"## 清洁日志消息\n"
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

  static const char* value(const ::hj_interface::NaviActionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
      stream.next(m.CleanRecord);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NaviActionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::NaviActionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::NaviActionResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
    s << indent << "CleanRecord: ";
    s << std::endl;
    Printer< ::hj_interface::CleanRecord_<ContainerAllocator> >::stream(s, indent + "  ", v.CleanRecord);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_NAVIACTIONRESPONSE_H
