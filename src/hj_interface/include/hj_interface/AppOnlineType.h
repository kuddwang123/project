// Generated by gencpp from file hj_interface/AppOnlineType.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_APPONLINETYPE_H
#define HJ_INTERFACE_MESSAGE_APPONLINETYPE_H


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
struct AppOnlineType_
{
  typedef AppOnlineType_<ContainerAllocator> Type;

  AppOnlineType_()
    : type(0)  {
    }
  AppOnlineType_(const ContainerAllocator& _alloc)
    : type(0)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(OFFLINE)
  #undef OFFLINE
#endif
#if defined(_WIN32) && defined(BT)
  #undef BT
#endif
#if defined(_WIN32) && defined(IOT)
  #undef IOT
#endif
#if defined(_WIN32) && defined(BT_IOT)
  #undef BT_IOT
#endif

  enum {
    OFFLINE = 0u,
    BT = 1u,
    IOT = 2u,
    BT_IOT = 3u,
  };


  typedef boost::shared_ptr< ::hj_interface::AppOnlineType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::AppOnlineType_<ContainerAllocator> const> ConstPtr;

}; // struct AppOnlineType_

typedef ::hj_interface::AppOnlineType_<std::allocator<void> > AppOnlineType;

typedef boost::shared_ptr< ::hj_interface::AppOnlineType > AppOnlineTypePtr;
typedef boost::shared_ptr< ::hj_interface::AppOnlineType const> AppOnlineTypeConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::AppOnlineType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::AppOnlineType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::AppOnlineType_<ContainerAllocator1> & lhs, const ::hj_interface::AppOnlineType_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::AppOnlineType_<ContainerAllocator1> & lhs, const ::hj_interface::AppOnlineType_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AppOnlineType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::AppOnlineType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AppOnlineType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::AppOnlineType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AppOnlineType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::AppOnlineType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::AppOnlineType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e6bd433252e9bcb153ba6ae5f28133e";
  }

  static const char* value(const ::hj_interface::AppOnlineType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e6bd433252e9bcbULL;
  static const uint64_t static_value2 = 0x153ba6ae5f28133eULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::AppOnlineType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/AppOnlineType";
  }

  static const char* value(const ::hj_interface::AppOnlineType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::AppOnlineType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8  OFFLINE  = 0\n"
"uint8  BT = 1\n"
"uint8  IOT = 2\n"
"uint8  BT_IOT = 3\n"
"uint8  type\n"
;
  }

  static const char* value(const ::hj_interface::AppOnlineType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::AppOnlineType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AppOnlineType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::AppOnlineType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::AppOnlineType_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_APPONLINETYPE_H
