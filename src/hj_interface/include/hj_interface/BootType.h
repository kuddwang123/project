// Generated by gencpp from file hj_interface/BootType.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_BOOTTYPE_H
#define HJ_INTERFACE_MESSAGE_BOOTTYPE_H


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
struct BootType_
{
  typedef BootType_<ContainerAllocator> Type;

  BootType_()
    : type(0)
    , ango()  {
    }
  BootType_(const ContainerAllocator& _alloc)
    : type(0)
    , ango(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _ango_type;
  _ango_type ango;





  typedef boost::shared_ptr< ::hj_interface::BootType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::BootType_<ContainerAllocator> const> ConstPtr;

}; // struct BootType_

typedef ::hj_interface::BootType_<std::allocator<void> > BootType;

typedef boost::shared_ptr< ::hj_interface::BootType > BootTypePtr;
typedef boost::shared_ptr< ::hj_interface::BootType const> BootTypeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::BootType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::BootType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::BootType_<ContainerAllocator1> & lhs, const ::hj_interface::BootType_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.ango == rhs.ango;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::BootType_<ContainerAllocator1> & lhs, const ::hj_interface::BootType_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::BootType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::BootType_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::BootType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::BootType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::BootType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::BootType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::BootType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9d72b3c84e110a521bdfcf76e3d2c80a";
  }

  static const char* value(const ::hj_interface::BootType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9d72b3c84e110a52ULL;
  static const uint64_t static_value2 = 0x1bdfcf76e3d2c80aULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::BootType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/BootType";
  }

  static const char* value(const ::hj_interface::BootType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::BootType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 type\n"
"string ango\n"
;
  }

  static const char* value(const ::hj_interface::BootType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::BootType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.ango);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BootType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::BootType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::BootType_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "ango: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.ango);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_BOOTTYPE_H
