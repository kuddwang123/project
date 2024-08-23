// Generated by gencpp from file collect_node/TwoIntsRequest.msg
// DO NOT EDIT!


#ifndef COLLECT_NODE_MESSAGE_TWOINTSREQUEST_H
#define COLLECT_NODE_MESSAGE_TWOINTSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace collect_node
{
template <class ContainerAllocator>
struct TwoIntsRequest_
{
  typedef TwoIntsRequest_<ContainerAllocator> Type;

  TwoIntsRequest_()
    : a(0)
    , b(0)  {
    }
  TwoIntsRequest_(const ContainerAllocator& _alloc)
    : a(0)
    , b(0)  {
  (void)_alloc;
    }



   typedef int64_t _a_type;
  _a_type a;

   typedef int64_t _b_type;
  _b_type b;





  typedef boost::shared_ptr< ::collect_node::TwoIntsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::collect_node::TwoIntsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TwoIntsRequest_

typedef ::collect_node::TwoIntsRequest_<std::allocator<void> > TwoIntsRequest;

typedef boost::shared_ptr< ::collect_node::TwoIntsRequest > TwoIntsRequestPtr;
typedef boost::shared_ptr< ::collect_node::TwoIntsRequest const> TwoIntsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::collect_node::TwoIntsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::collect_node::TwoIntsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::collect_node::TwoIntsRequest_<ContainerAllocator1> & lhs, const ::collect_node::TwoIntsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.a == rhs.a &&
    lhs.b == rhs.b;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::collect_node::TwoIntsRequest_<ContainerAllocator1> & lhs, const ::collect_node::TwoIntsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace collect_node

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::collect_node::TwoIntsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::collect_node::TwoIntsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::collect_node::TwoIntsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "36d09b846be0b371c5f190354dd3153e";
  }

  static const char* value(const ::collect_node::TwoIntsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x36d09b846be0b371ULL;
  static const uint64_t static_value2 = 0xc5f190354dd3153eULL;
};

template<class ContainerAllocator>
struct DataType< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "collect_node/TwoIntsRequest";
  }

  static const char* value(const ::collect_node::TwoIntsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 a\n"
"int64 b\n"
;
  }

  static const char* value(const ::collect_node::TwoIntsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TwoIntsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::collect_node::TwoIntsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::collect_node::TwoIntsRequest_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<int64_t>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<int64_t>::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COLLECT_NODE_MESSAGE_TWOINTSREQUEST_H