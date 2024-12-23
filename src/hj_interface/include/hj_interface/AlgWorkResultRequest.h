// Generated by gencpp from file hj_interface/AlgWorkResultRequest.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_ALGWORKRESULTREQUEST_H
#define HJ_INTERFACE_MESSAGE_ALGWORKRESULTREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace hj_interface
{
template <class ContainerAllocator>
struct AlgWorkResultRequest_
{
  typedef AlgWorkResultRequest_<ContainerAllocator> Type;

  AlgWorkResultRequest_()
    : action_cmd(0)
    , action_result(0)
    , point()  {
    }
  AlgWorkResultRequest_(const ContainerAllocator& _alloc)
    : action_cmd(0)
    , action_result(0)
    , point(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _action_cmd_type;
  _action_cmd_type action_cmd;

   typedef uint8_t _action_result_type;
  _action_result_type action_result;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point_type;
  _point_type point;





  typedef boost::shared_ptr< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> const> ConstPtr;

}; // struct AlgWorkResultRequest_

typedef ::hj_interface::AlgWorkResultRequest_<std::allocator<void> > AlgWorkResultRequest;

typedef boost::shared_ptr< ::hj_interface::AlgWorkResultRequest > AlgWorkResultRequestPtr;
typedef boost::shared_ptr< ::hj_interface::AlgWorkResultRequest const> AlgWorkResultRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator1> & lhs, const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator2> & rhs)
{
  return lhs.action_cmd == rhs.action_cmd &&
    lhs.action_result == rhs.action_result &&
    lhs.point == rhs.point;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator1> & lhs, const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb2ad21af61085b829d1fd3723bc6847";
  }

  static const char* value(const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb2ad21af61085b8ULL;
  static const uint64_t static_value2 = 0x29d1fd3723bc6847ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/AlgWorkResultRequest";
  }

  static const char* value(const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 action_cmd            # 26: 召回重定位\n"
"\n"
"uint8 action_result         # 1: 召回重定位成功; 2: 召回重定位失败;\n"
"\n"
"geometry_msgs/Point point  # 入水点\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_cmd);
      stream.next(m.action_result);
      stream.next(m.point);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AlgWorkResultRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::AlgWorkResultRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::AlgWorkResultRequest_<ContainerAllocator>& v)
  {
    s << indent << "action_cmd: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_cmd);
    s << indent << "action_result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_result);
    s << indent << "point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_ALGWORKRESULTREQUEST_H
