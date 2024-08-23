// Generated by gencpp from file hj_interface/AirBag.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_AIRBAG_H
#define HJ_INTERFACE_MESSAGE_AIRBAG_H


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
struct AirBag_
{
  typedef AirBag_<ContainerAllocator> Type;

  AirBag_()
    : timestamp()
    , airbag_ctl(0)
    , airbag_time(0)  {
    }
  AirBag_(const ContainerAllocator& _alloc)
    : timestamp()
    , airbag_ctl(0)
    , airbag_time(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _airbag_ctl_type;
  _airbag_ctl_type airbag_ctl;

   typedef uint16_t _airbag_time_type;
  _airbag_time_type airbag_time;





  typedef boost::shared_ptr< ::hj_interface::AirBag_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::AirBag_<ContainerAllocator> const> ConstPtr;

}; // struct AirBag_

typedef ::hj_interface::AirBag_<std::allocator<void> > AirBag;

typedef boost::shared_ptr< ::hj_interface::AirBag > AirBagPtr;
typedef boost::shared_ptr< ::hj_interface::AirBag const> AirBagConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::AirBag_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::AirBag_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::AirBag_<ContainerAllocator1> & lhs, const ::hj_interface::AirBag_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.airbag_ctl == rhs.airbag_ctl &&
    lhs.airbag_time == rhs.airbag_time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::AirBag_<ContainerAllocator1> & lhs, const ::hj_interface::AirBag_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AirBag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AirBag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AirBag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AirBag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AirBag_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AirBag_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::AirBag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7373b3ba46c833135f263694b4539235";
  }

  static const char* value(const ::hj_interface::AirBag_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7373b3ba46c83313ULL;
  static const uint64_t static_value2 = 0x5f263694b4539235ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::AirBag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/AirBag";
  }

  static const char* value(const ::hj_interface::AirBag_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::AirBag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n"
"uint8 airbag_ctl     #!< 0 全放气； 1 全充气； 2 左充气；  3 右充气 ； 气囊控制   \n"
"uint16 airbag_time      #!< 充/放气时间  单位10ms\n"
;
  }

  static const char* value(const ::hj_interface::AirBag_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::AirBag_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.airbag_ctl);
      stream.next(m.airbag_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AirBag_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::AirBag_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::AirBag_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "airbag_ctl: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.airbag_ctl);
    s << indent << "airbag_time: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.airbag_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_AIRBAG_H
