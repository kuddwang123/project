// Generated by gencpp from file hj_interface/MagCalibActionResponse.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_MAGCALIBACTIONRESPONSE_H
#define HJ_INTERFACE_MESSAGE_MAGCALIBACTIONRESPONSE_H


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
struct MagCalibActionResponse_
{
  typedef MagCalibActionResponse_<ContainerAllocator> Type;

  MagCalibActionResponse_()
    : success(false)  {
    }
  MagCalibActionResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MagCalibActionResponse_

typedef ::hj_interface::MagCalibActionResponse_<std::allocator<void> > MagCalibActionResponse;

typedef boost::shared_ptr< ::hj_interface::MagCalibActionResponse > MagCalibActionResponsePtr;
typedef boost::shared_ptr< ::hj_interface::MagCalibActionResponse const> MagCalibActionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::MagCalibActionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::MagCalibActionResponse_<ContainerAllocator1> & lhs, const ::hj_interface::MagCalibActionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::MagCalibActionResponse_<ContainerAllocator1> & lhs, const ::hj_interface::MagCalibActionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::hj_interface::MagCalibActionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/MagCalibActionResponse";
  }

  static const char* value(const ::hj_interface::MagCalibActionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"bool success # 0 标定失败 1 标定完成\n"
;
  }

  static const char* value(const ::hj_interface::MagCalibActionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MagCalibActionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::MagCalibActionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::MagCalibActionResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_MAGCALIBACTIONRESPONSE_H
