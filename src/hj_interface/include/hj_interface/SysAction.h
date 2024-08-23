// Generated by gencpp from file hj_interface/SysAction.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_SYSACTION_H
#define HJ_INTERFACE_MESSAGE_SYSACTION_H


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
struct SysAction_
{
  typedef SysAction_<ContainerAllocator> Type;

  SysAction_()
    : action(0)
    , res(0)
    , from(0)  {
    }
  SysAction_(const ContainerAllocator& _alloc)
    : action(0)
    , res(0)
    , from(0)  {
  (void)_alloc;
    }



   typedef uint8_t _action_type;
  _action_type action;

   typedef uint8_t _res_type;
  _res_type res;

   typedef uint8_t _from_type;
  _from_type from;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(SYS_ACTION_SHUTDOWN)
  #undef SYS_ACTION_SHUTDOWN
#endif
#if defined(_WIN32) && defined(SYS_ACTION_RESET)
  #undef SYS_ACTION_RESET
#endif
#if defined(_WIN32) && defined(SYS_ACTION_RESTORE_FACTORY)
  #undef SYS_ACTION_RESTORE_FACTORY
#endif
#if defined(_WIN32) && defined(SYS_ACTION_SLEEP)
  #undef SYS_ACTION_SLEEP
#endif
#if defined(_WIN32) && defined(COMM_NODE_COLLECT)
  #undef COMM_NODE_COLLECT
#endif
#if defined(_WIN32) && defined(COMM_NODE_MIDDLEWARE)
  #undef COMM_NODE_MIDDLEWARE
#endif
#if defined(_WIN32) && defined(COMM_NODE_PLANNING)
  #undef COMM_NODE_PLANNING
#endif
#if defined(_WIN32) && defined(COMM_NODE_SLAM)
  #undef COMM_NODE_SLAM
#endif
#if defined(_WIN32) && defined(COMM_NODE_UTILS)
  #undef COMM_NODE_UTILS
#endif
#if defined(_WIN32) && defined(COMM_NODE_OS)
  #undef COMM_NODE_OS
#endif

  enum {
    SYS_ACTION_SHUTDOWN = 0u,
    SYS_ACTION_RESET = 1u,
    SYS_ACTION_RESTORE_FACTORY = 2u,
    SYS_ACTION_SLEEP = 3u,
    COMM_NODE_COLLECT = 1u,
    COMM_NODE_MIDDLEWARE = 2u,
    COMM_NODE_PLANNING = 3u,
    COMM_NODE_SLAM = 4u,
    COMM_NODE_UTILS = 5u,
    COMM_NODE_OS = 6u,
  };


  typedef boost::shared_ptr< ::hj_interface::SysAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::SysAction_<ContainerAllocator> const> ConstPtr;

}; // struct SysAction_

typedef ::hj_interface::SysAction_<std::allocator<void> > SysAction;

typedef boost::shared_ptr< ::hj_interface::SysAction > SysActionPtr;
typedef boost::shared_ptr< ::hj_interface::SysAction const> SysActionConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::SysAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::SysAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::SysAction_<ContainerAllocator1> & lhs, const ::hj_interface::SysAction_<ContainerAllocator2> & rhs)
{
  return lhs.action == rhs.action &&
    lhs.res == rhs.res &&
    lhs.from == rhs.from;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::SysAction_<ContainerAllocator1> & lhs, const ::hj_interface::SysAction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::SysAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::SysAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::SysAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::SysAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::SysAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::SysAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::SysAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c027e147647b0b2126e520c494a0384d";
  }

  static const char* value(const ::hj_interface::SysAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc027e147647b0b21ULL;
  static const uint64_t static_value2 = 0x26e520c494a0384dULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::SysAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/SysAction";
  }

  static const char* value(const ::hj_interface::SysAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::SysAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 action    # system action type: see enum SYS_ACTION_*\n"
"uint8 res       # result: 0 means OK\n"
"uint8 from      # source node : see enum COMM_NODE_*\n"
"uint8 SYS_ACTION_SHUTDOWN = 0\n"
"uint8 SYS_ACTION_RESET = 1      # reset from remote control\n"
"uint8 SYS_ACTION_RESTORE_FACTORY = 2    # restore factory from key\n"
"uint8 SYS_ACTION_SLEEP = 3\n"
"uint8 COMM_NODE_COLLECT = 1\n"
"uint8 COMM_NODE_MIDDLEWARE = 2\n"
"uint8 COMM_NODE_PLANNING = 3\n"
"uint8 COMM_NODE_SLAM = 4\n"
"uint8 COMM_NODE_UTILS = 5\n"
"uint8 COMM_NODE_OS = 6\n"
;
  }

  static const char* value(const ::hj_interface::SysAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::SysAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action);
      stream.next(m.res);
      stream.next(m.from);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SysAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::SysAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::SysAction_<ContainerAllocator>& v)
  {
    s << indent << "action: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action);
    s << indent << "res: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.res);
    s << indent << "from: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.from);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_SYSACTION_H
