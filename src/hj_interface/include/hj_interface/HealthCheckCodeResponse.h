// Generated by gencpp from file hj_interface/HealthCheckCodeResponse.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_HEALTHCHECKCODERESPONSE_H
#define HJ_INTERFACE_MESSAGE_HEALTHCHECKCODERESPONSE_H


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
struct HealthCheckCodeResponse_
{
  typedef HealthCheckCodeResponse_<ContainerAllocator> Type;

  HealthCheckCodeResponse_()
    : result(0)  {
    }
  HealthCheckCodeResponse_(const ContainerAllocator& _alloc)
    : result(0)  {
  (void)_alloc;
    }



   typedef uint8_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct HealthCheckCodeResponse_

typedef ::hj_interface::HealthCheckCodeResponse_<std::allocator<void> > HealthCheckCodeResponse;

typedef boost::shared_ptr< ::hj_interface::HealthCheckCodeResponse > HealthCheckCodeResponsePtr;
typedef boost::shared_ptr< ::hj_interface::HealthCheckCodeResponse const> HealthCheckCodeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator1> & lhs, const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator1> & lhs, const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "25458147911545c320c4c0a299eff763";
  }

  static const char* value(const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x25458147911545c3ULL;
  static const uint64_t static_value2 = 0x20c4c0a299eff763ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/HealthCheckCodeResponse";
  }

  static const char* value(const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 result   # 1: receive\n"
;
  }

  static const char* value(const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HealthCheckCodeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::HealthCheckCodeResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_HEALTHCHECKCODERESPONSE_H
