// Generated by gencpp from file hj_interface/AbnormalState.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_ABNORMALSTATE_H
#define HJ_INTERFACE_MESSAGE_ABNORMALSTATE_H


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
struct AbnormalState_
{
  typedef AbnormalState_<ContainerAllocator> Type;

  AbnormalState_()
    : abnormal_states()  {
    }
  AbnormalState_(const ContainerAllocator& _alloc)
    : abnormal_states(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _abnormal_states_type;
  _abnormal_states_type abnormal_states;





  typedef boost::shared_ptr< ::hj_interface::AbnormalState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::AbnormalState_<ContainerAllocator> const> ConstPtr;

}; // struct AbnormalState_

typedef ::hj_interface::AbnormalState_<std::allocator<void> > AbnormalState;

typedef boost::shared_ptr< ::hj_interface::AbnormalState > AbnormalStatePtr;
typedef boost::shared_ptr< ::hj_interface::AbnormalState const> AbnormalStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::AbnormalState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::AbnormalState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::AbnormalState_<ContainerAllocator1> & lhs, const ::hj_interface::AbnormalState_<ContainerAllocator2> & rhs)
{
  return lhs.abnormal_states == rhs.abnormal_states;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::AbnormalState_<ContainerAllocator1> & lhs, const ::hj_interface::AbnormalState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AbnormalState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AbnormalState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AbnormalState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AbnormalState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AbnormalState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AbnormalState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::AbnormalState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1183477c095c5373f9f3fd7dda654f5e";
  }

  static const char* value(const ::hj_interface::AbnormalState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1183477c095c5373ULL;
  static const uint64_t static_value2 = 0xf9f3fd7dda654f5eULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::AbnormalState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/AbnormalState";
  }

  static const char* value(const ::hj_interface::AbnormalState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::AbnormalState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] abnormal_states\n"
;
  }

  static const char* value(const ::hj_interface::AbnormalState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::AbnormalState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.abnormal_states);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AbnormalState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::AbnormalState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::AbnormalState_<ContainerAllocator>& v)
  {
    s << indent << "abnormal_states[]" << std::endl;
    for (size_t i = 0; i < v.abnormal_states.size(); ++i)
    {
      s << indent << "  abnormal_states[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.abnormal_states[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_ABNORMALSTATE_H