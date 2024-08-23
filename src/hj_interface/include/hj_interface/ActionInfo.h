// Generated by gencpp from file hj_interface/ActionInfo.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_ACTIONINFO_H
#define HJ_INTERFACE_MESSAGE_ACTIONINFO_H


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
struct ActionInfo_
{
  typedef ActionInfo_<ContainerAllocator> Type;

  ActionInfo_()
    : action_cmd(0)
    , clean_mode(0)  {
    }
  ActionInfo_(const ContainerAllocator& _alloc)
    : action_cmd(0)
    , clean_mode(0)  {
  (void)_alloc;
    }



   typedef int32_t _action_cmd_type;
  _action_cmd_type action_cmd;

   typedef int32_t _clean_mode_type;
  _clean_mode_type clean_mode;





  typedef boost::shared_ptr< ::hj_interface::ActionInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::ActionInfo_<ContainerAllocator> const> ConstPtr;

}; // struct ActionInfo_

typedef ::hj_interface::ActionInfo_<std::allocator<void> > ActionInfo;

typedef boost::shared_ptr< ::hj_interface::ActionInfo > ActionInfoPtr;
typedef boost::shared_ptr< ::hj_interface::ActionInfo const> ActionInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::ActionInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::ActionInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::ActionInfo_<ContainerAllocator1> & lhs, const ::hj_interface::ActionInfo_<ContainerAllocator2> & rhs)
{
  return lhs.action_cmd == rhs.action_cmd &&
    lhs.clean_mode == rhs.clean_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::ActionInfo_<ContainerAllocator1> & lhs, const ::hj_interface::ActionInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::ActionInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::ActionInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::ActionInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::ActionInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::ActionInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::ActionInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::ActionInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "70058255d47a4aad6b3ce4135c5145e4";
  }

  static const char* value(const ::hj_interface::ActionInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x70058255d47a4aadULL;
  static const uint64_t static_value2 = 0x6b3ce4135c5145e4ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::ActionInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/ActionInfo";
  }

  static const char* value(const ::hj_interface::ActionInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::ActionInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 action_cmd    # 1-水面 2-池底 3-池壁 \n"
"                    # 11-建图 12-定位  \n"
"                    # 21-召回 22-回充 \n"
"                    # 31-导航状态查询 36-清洁模式\n"
"                    # 101-pause, 102-resume, 103-stop, 104-fail\n"
"int32 clean_mode    # 1-变频清洁 2-标准清洁 3-深度清洁\n"
;
  }

  static const char* value(const ::hj_interface::ActionInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::ActionInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_cmd);
      stream.next(m.clean_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActionInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::ActionInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::ActionInfo_<ContainerAllocator>& v)
  {
    s << indent << "action_cmd: ";
    Printer<int32_t>::stream(s, indent + "  ", v.action_cmd);
    s << indent << "clean_mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.clean_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_ACTIONINFO_H
