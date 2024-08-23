// Generated by gencpp from file hj_interface/Depth.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_DEPTH_H
#define HJ_INTERFACE_MESSAGE_DEPTH_H


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
struct Depth_
{
  typedef Depth_<ContainerAllocator> Type;

  Depth_()
    : timestamp()
    , pressure(0)
    , temp(0)  {
    }
  Depth_(const ContainerAllocator& _alloc)
    : timestamp()
    , pressure(0)
    , temp(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef int32_t _pressure_type;
  _pressure_type pressure;

   typedef int32_t _temp_type;
  _temp_type temp;





  typedef boost::shared_ptr< ::hj_interface::Depth_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::Depth_<ContainerAllocator> const> ConstPtr;

}; // struct Depth_

typedef ::hj_interface::Depth_<std::allocator<void> > Depth;

typedef boost::shared_ptr< ::hj_interface::Depth > DepthPtr;
typedef boost::shared_ptr< ::hj_interface::Depth const> DepthConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::Depth_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::Depth_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::Depth_<ContainerAllocator1> & lhs, const ::hj_interface::Depth_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.pressure == rhs.pressure &&
    lhs.temp == rhs.temp;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::Depth_<ContainerAllocator1> & lhs, const ::hj_interface::Depth_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::Depth_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::Depth_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::Depth_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::Depth_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::Depth_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::Depth_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::Depth_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1bbaa718774b285065f3e36f3c8f8636";
  }

  static const char* value(const ::hj_interface::Depth_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1bbaa718774b2850ULL;
  static const uint64_t static_value2 = 0x65f3e36f3c8f8636ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::Depth_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/Depth";
  }

  static const char* value(const ::hj_interface::Depth_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::Depth_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp #!< 时间戳，单位us\n"
"int32 pressure # !< 压力，单位pa\n"
"int32 temp  # !< 温度，单位摄氏度\n"
;
  }

  static const char* value(const ::hj_interface::Depth_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::Depth_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.pressure);
      stream.next(m.temp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Depth_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::Depth_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::Depth_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "pressure: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pressure);
    s << indent << "temp: ";
    Printer<int32_t>::stream(s, indent + "  ", v.temp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_DEPTH_H