// Generated by gencpp from file hj_interface/DownRight.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_DOWNRIGHT_H
#define HJ_INTERFACE_MESSAGE_DOWNRIGHT_H


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
struct DownRight_
{
  typedef DownRight_<ContainerAllocator> Type;

  DownRight_()
    : timestamp()
    , dist(0)
    , status(0)  {
    }
  DownRight_(const ContainerAllocator& _alloc)
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





  typedef boost::shared_ptr< ::hj_interface::DownRight_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::DownRight_<ContainerAllocator> const> ConstPtr;

}; // struct DownRight_

typedef ::hj_interface::DownRight_<std::allocator<void> > DownRight;

typedef boost::shared_ptr< ::hj_interface::DownRight > DownRightPtr;
typedef boost::shared_ptr< ::hj_interface::DownRight const> DownRightConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::DownRight_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::DownRight_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::DownRight_<ContainerAllocator1> & lhs, const ::hj_interface::DownRight_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.dist == rhs.dist &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::DownRight_<ContainerAllocator1> & lhs, const ::hj_interface::DownRight_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::DownRight_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::DownRight_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::DownRight_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::DownRight_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::DownRight_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::DownRight_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::DownRight_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a9d834c76ccc6547fc68461a601bdec2";
  }

  static const char* value(const ::hj_interface::DownRight_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa9d834c76ccc6547ULL;
  static const uint64_t static_value2 = 0xfc68461a601bdec2ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::DownRight_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/DownRight";
  }

  static const char* value(const ::hj_interface::DownRight_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::DownRight_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp \n"
"uint32 dist  #!< 下面超声波距离,单位:mm.\n"
"uint8 status #!< 超声波状态,0 means normal,1 means error.\n"
;
  }

  static const char* value(const ::hj_interface::DownRight_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::DownRight_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.dist);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DownRight_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::DownRight_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::DownRight_<ContainerAllocator>& v)
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

#endif // HJ_INTERFACE_MESSAGE_DOWNRIGHT_H
