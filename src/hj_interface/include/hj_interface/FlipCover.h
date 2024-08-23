// Generated by gencpp from file hj_interface/FlipCover.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_FLIPCOVER_H
#define HJ_INTERFACE_MESSAGE_FLIPCOVER_H


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
struct FlipCover_
{
  typedef FlipCover_<ContainerAllocator> Type;

  FlipCover_()
    : timestamp()
    , flip_cover(0)  {
    }
  FlipCover_(const ContainerAllocator& _alloc)
    : timestamp()
    , flip_cover(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint8_t _flip_cover_type;
  _flip_cover_type flip_cover;





  typedef boost::shared_ptr< ::hj_interface::FlipCover_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::FlipCover_<ContainerAllocator> const> ConstPtr;

}; // struct FlipCover_

typedef ::hj_interface::FlipCover_<std::allocator<void> > FlipCover;

typedef boost::shared_ptr< ::hj_interface::FlipCover > FlipCoverPtr;
typedef boost::shared_ptr< ::hj_interface::FlipCover const> FlipCoverConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::FlipCover_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::FlipCover_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::FlipCover_<ContainerAllocator1> & lhs, const ::hj_interface::FlipCover_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.flip_cover == rhs.flip_cover;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::FlipCover_<ContainerAllocator1> & lhs, const ::hj_interface::FlipCover_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::FlipCover_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::FlipCover_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::FlipCover_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::FlipCover_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::FlipCover_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::FlipCover_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::FlipCover_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7be9610adccf93d711d3952a805e70a5";
  }

  static const char* value(const ::hj_interface::FlipCover_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7be9610adccf93d7ULL;
  static const uint64_t static_value2 = 0x11d3952a805e70a5ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::FlipCover_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/FlipCover";
  }

  static const char* value(const ::hj_interface::FlipCover_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::FlipCover_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n"
"uint8 flip_cover    #!< 翻盖电机  位置  0:关闭, 1:打开 上吸口开门\n"
;
  }

  static const char* value(const ::hj_interface::FlipCover_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::FlipCover_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.flip_cover);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FlipCover_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::FlipCover_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::FlipCover_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "flip_cover: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flip_cover);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_FLIPCOVER_H
