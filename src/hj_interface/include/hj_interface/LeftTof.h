// Generated by gencpp from file hj_interface/LeftTof.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_LEFTTOF_H
#define HJ_INTERFACE_MESSAGE_LEFTTOF_H


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
struct LeftTof_
{
  typedef LeftTof_<ContainerAllocator> Type;

  LeftTof_()
    : timestamp()
    , dist_front(0)
    , dist_back(0)  {
    }
  LeftTof_(const ContainerAllocator& _alloc)
    : timestamp()
    , dist_front(0)
    , dist_back(0)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef uint32_t _dist_front_type;
  _dist_front_type dist_front;

   typedef uint32_t _dist_back_type;
  _dist_back_type dist_back;





  typedef boost::shared_ptr< ::hj_interface::LeftTof_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::LeftTof_<ContainerAllocator> const> ConstPtr;

}; // struct LeftTof_

typedef ::hj_interface::LeftTof_<std::allocator<void> > LeftTof;

typedef boost::shared_ptr< ::hj_interface::LeftTof > LeftTofPtr;
typedef boost::shared_ptr< ::hj_interface::LeftTof const> LeftTofConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::LeftTof_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::LeftTof_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::LeftTof_<ContainerAllocator1> & lhs, const ::hj_interface::LeftTof_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.dist_front == rhs.dist_front &&
    lhs.dist_back == rhs.dist_back;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::LeftTof_<ContainerAllocator1> & lhs, const ::hj_interface::LeftTof_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::LeftTof_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::LeftTof_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::LeftTof_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::LeftTof_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::LeftTof_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::LeftTof_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::LeftTof_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ba4ea4c58d000312f1be33b3dcae86e1";
  }

  static const char* value(const ::hj_interface::LeftTof_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xba4ea4c58d000312ULL;
  static const uint64_t static_value2 = 0xf1be33b3dcae86e1ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::LeftTof_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/LeftTof";
  }

  static const char* value(const ::hj_interface::LeftTof_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::LeftTof_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n"
"uint32 dist_front    # !< 激光测距，左前激光探测距离，单位：mm\n"
"uint32 dist_back     # !< 激光测距，左后激光探测距离，单位：mm\n"
;
  }

  static const char* value(const ::hj_interface::LeftTof_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::LeftTof_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.dist_front);
      stream.next(m.dist_back);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LeftTof_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::LeftTof_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::LeftTof_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "dist_front: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.dist_front);
    s << indent << "dist_back: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.dist_back);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_LEFTTOF_H
