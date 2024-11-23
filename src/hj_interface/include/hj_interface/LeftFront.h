// Generated by gencpp from file hj_interface/LeftFront.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_LEFTFRONT_H
#define HJ_INTERFACE_MESSAGE_LEFTFRONT_H


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
struct LeftFront_
{
  typedef LeftFront_<ContainerAllocator> Type;

  LeftFront_()
    : timestamp()
    , dist(0)
    , status(0)  {
    }
  LeftFront_(const ContainerAllocator& _alloc)
    : timestamp()
    , dist(0)
    , status(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint32_t _dist_type;
  _dist_type dist;

   typedef uint8_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::hj_interface::LeftFront_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::LeftFront_<ContainerAllocator> const> ConstPtr;

}; // struct LeftFront_

typedef ::hj_interface::LeftFront_<std::allocator<void> > LeftFront;

typedef boost::shared_ptr< ::hj_interface::LeftFront > LeftFrontPtr;
typedef boost::shared_ptr< ::hj_interface::LeftFront const> LeftFrontConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::LeftFront_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::LeftFront_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::LeftFront_<ContainerAllocator1> & lhs, const ::hj_interface::LeftFront_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.dist == rhs.dist &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::LeftFront_<ContainerAllocator1> & lhs, const ::hj_interface::LeftFront_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::LeftFront_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::LeftFront_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::LeftFront_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::LeftFront_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::LeftFront_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::LeftFront_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::LeftFront_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a9d834c76ccc6547fc68461a601bdec2";
  }

  static const char* value(const ::hj_interface::LeftFront_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa9d834c76ccc6547ULL;
  static const uint64_t static_value2 = 0xfc68461a601bdec2ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::LeftFront_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/LeftFront";
  }

  static const char* value(const ::hj_interface::LeftFront_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::LeftFront_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp \n"
"uint32 dist  #!< 左前超声波距离,单位:mm.\n"
"uint8 status #!< 超声波状态,0 means normal,1 means error, 2 means 传感器出水..\n"
;
  }

  static const char* value(const ::hj_interface::LeftFront_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::LeftFront_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.dist);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LeftFront_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::LeftFront_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::LeftFront_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "dist: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.dist);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_LEFTFRONT_H
