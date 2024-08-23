// Generated by gencpp from file hj_interface/ImuWorkModel.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_IMUWORKMODEL_H
#define HJ_INTERFACE_MESSAGE_IMUWORKMODEL_H


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
struct ImuWorkModel_
{
  typedef ImuWorkModel_<ContainerAllocator> Type;

  ImuWorkModel_()
    : custom_time()
    , model(0)
    , type(0)  {
    }
  ImuWorkModel_(const ContainerAllocator& _alloc)
    : custom_time()
    , model(0)
    , type(0)  {
  (void)_alloc;
    }



   typedef ros::Time _custom_time_type;
  _custom_time_type custom_time;

   typedef uint8_t _model_type;
  _model_type model;

   typedef uint8_t _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::hj_interface::ImuWorkModel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::ImuWorkModel_<ContainerAllocator> const> ConstPtr;

}; // struct ImuWorkModel_

typedef ::hj_interface::ImuWorkModel_<std::allocator<void> > ImuWorkModel;

typedef boost::shared_ptr< ::hj_interface::ImuWorkModel > ImuWorkModelPtr;
typedef boost::shared_ptr< ::hj_interface::ImuWorkModel const> ImuWorkModelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::ImuWorkModel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::ImuWorkModel_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::ImuWorkModel_<ContainerAllocator1> & lhs, const ::hj_interface::ImuWorkModel_<ContainerAllocator2> & rhs)
{
  return lhs.custom_time == rhs.custom_time &&
    lhs.model == rhs.model &&
    lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::ImuWorkModel_<ContainerAllocator1> & lhs, const ::hj_interface::ImuWorkModel_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::ImuWorkModel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::ImuWorkModel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::ImuWorkModel_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bc97664345f428c8b872445a9621d5d8";
  }

  static const char* value(const ::hj_interface::ImuWorkModel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbc97664345f428c8ULL;
  static const uint64_t static_value2 = 0xb872445a9621d5d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/ImuWorkModel";
  }

  static const char* value(const ::hj_interface::ImuWorkModel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time custom_time\n"
"uint8 model # 1-池底 2-池壁 3-水线 4-水面 5-台阶\n"
"uint8 type  # 0-关闭 1-开启 2-正在\n"
;
  }

  static const char* value(const ::hj_interface::ImuWorkModel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.custom_time);
      stream.next(m.model);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ImuWorkModel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::ImuWorkModel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::ImuWorkModel_<ContainerAllocator>& v)
  {
    s << indent << "custom_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.custom_time);
    s << indent << "model: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.model);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_IMUWORKMODEL_H
