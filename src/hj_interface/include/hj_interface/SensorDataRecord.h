// Generated by gencpp from file hj_interface/SensorDataRecord.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_SENSORDATARECORD_H
#define HJ_INTERFACE_MESSAGE_SENSORDATARECORD_H


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
struct SensorDataRecord_
{
  typedef SensorDataRecord_<ContainerAllocator> Type;

  SensorDataRecord_()
    : action_cmd(0)  {
    }
  SensorDataRecord_(const ContainerAllocator& _alloc)
    : action_cmd(0)  {
  (void)_alloc;
    }



   typedef uint8_t _action_cmd_type;
  _action_cmd_type action_cmd;





  typedef boost::shared_ptr< ::hj_interface::SensorDataRecord_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::SensorDataRecord_<ContainerAllocator> const> ConstPtr;

}; // struct SensorDataRecord_

typedef ::hj_interface::SensorDataRecord_<std::allocator<void> > SensorDataRecord;

typedef boost::shared_ptr< ::hj_interface::SensorDataRecord > SensorDataRecordPtr;
typedef boost::shared_ptr< ::hj_interface::SensorDataRecord const> SensorDataRecordConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::SensorDataRecord_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::SensorDataRecord_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::SensorDataRecord_<ContainerAllocator1> & lhs, const ::hj_interface::SensorDataRecord_<ContainerAllocator2> & rhs)
{
  return lhs.action_cmd == rhs.action_cmd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::SensorDataRecord_<ContainerAllocator1> & lhs, const ::hj_interface::SensorDataRecord_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::SensorDataRecord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::SensorDataRecord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::SensorDataRecord_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f5f59ca82f166afa80831a42f295d78";
  }

  static const char* value(const ::hj_interface::SensorDataRecord_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f5f59ca82f166afULL;
  static const uint64_t static_value2 = 0xa80831a42f295d78ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/SensorDataRecord";
  }

  static const char* value(const ::hj_interface::SensorDataRecord_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 action_cmd    # 11: 水面任务  13: 池壁任务 14：水线任务 15：池底任务\n"
"                    # 35: 结束当前任务录制\n"
;
  }

  static const char* value(const ::hj_interface::SensorDataRecord_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SensorDataRecord_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::SensorDataRecord_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::SensorDataRecord_<ContainerAllocator>& v)
  {
    s << indent << "action_cmd: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_SENSORDATARECORD_H
