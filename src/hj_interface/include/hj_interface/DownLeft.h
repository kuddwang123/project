// Generated by gencpp from file hj_interface/DownLeft.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_DOWNLEFT_H
#define HJ_INTERFACE_MESSAGE_DOWNLEFT_H


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
struct DownLeft_
{
  typedef DownLeft_<ContainerAllocator> Type;

  DownLeft_()
    : timestamp()
    , ray_value(0)  {
    }
  DownLeft_(const ContainerAllocator& _alloc)
    : timestamp()
    , ray_value(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _ray_value_type;
  _ray_value_type ray_value;





  typedef boost::shared_ptr< ::hj_interface::DownLeft_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::DownLeft_<ContainerAllocator> const> ConstPtr;

}; // struct DownLeft_

typedef ::hj_interface::DownLeft_<std::allocator<void> > DownLeft;

typedef boost::shared_ptr< ::hj_interface::DownLeft > DownLeftPtr;
typedef boost::shared_ptr< ::hj_interface::DownLeft const> DownLeftConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::DownLeft_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::DownLeft_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::DownLeft_<ContainerAllocator1> & lhs, const ::hj_interface::DownLeft_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.ray_value == rhs.ray_value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::DownLeft_<ContainerAllocator1> & lhs, const ::hj_interface::DownLeft_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::DownLeft_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::DownLeft_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::DownLeft_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::DownLeft_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::DownLeft_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::DownLeft_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::DownLeft_<ContainerAllocator> >
{
  static const char* value()
  {
    return "325366b738b13afa660bf980f3269583";
  }

  static const char* value(const ::hj_interface::DownLeft_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x325366b738b13afaULL;
  static const uint64_t static_value2 = 0x660bf980f3269583ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::DownLeft_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/DownLeft";
  }

  static const char* value(const ::hj_interface::DownLeft_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::DownLeft_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp \n"
"uint8 ray_value  #!< 下视红外， 0: 有遮挡, 1: 无遮挡.\n"
;
  }

  static const char* value(const ::hj_interface::DownLeft_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::DownLeft_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.ray_value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DownLeft_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::DownLeft_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::DownLeft_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "ray_value: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ray_value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_DOWNLEFT_H