// Generated by gencpp from file hj_interface/StationUltra.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_STATIONULTRA_H
#define HJ_INTERFACE_MESSAGE_STATIONULTRA_H


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
struct StationUltra_
{
  typedef StationUltra_<ContainerAllocator> Type;

  StationUltra_()
    : ultra_lt()
    , ultra_rt()  {
    }
  StationUltra_(const ContainerAllocator& _alloc)
    : ultra_lt(_alloc)
    , ultra_rt(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> _ultra_lt_type;
  _ultra_lt_type ultra_lt;

   typedef std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> _ultra_rt_type;
  _ultra_rt_type ultra_rt;





  typedef boost::shared_ptr< ::hj_interface::StationUltra_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::StationUltra_<ContainerAllocator> const> ConstPtr;

}; // struct StationUltra_

typedef ::hj_interface::StationUltra_<std::allocator<void> > StationUltra;

typedef boost::shared_ptr< ::hj_interface::StationUltra > StationUltraPtr;
typedef boost::shared_ptr< ::hj_interface::StationUltra const> StationUltraConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::StationUltra_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::StationUltra_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::StationUltra_<ContainerAllocator1> & lhs, const ::hj_interface::StationUltra_<ContainerAllocator2> & rhs)
{
  return lhs.ultra_lt == rhs.ultra_lt &&
    lhs.ultra_rt == rhs.ultra_rt;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::StationUltra_<ContainerAllocator1> & lhs, const ::hj_interface::StationUltra_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::StationUltra_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::StationUltra_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::StationUltra_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::StationUltra_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::StationUltra_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::StationUltra_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::StationUltra_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8f9efa65f662a357f43077cc584d92e8";
  }

  static const char* value(const ::hj_interface::StationUltra_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8f9efa65f662a357ULL;
  static const uint64_t static_value2 = 0xf43077cc584d92e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::StationUltra_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/StationUltra";
  }

  static const char* value(const ::hj_interface::StationUltra_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::StationUltra_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16[]  ultra_lt\n"
"uint16[]  ultra_rt\n"
;
  }

  static const char* value(const ::hj_interface::StationUltra_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::StationUltra_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ultra_lt);
      stream.next(m.ultra_rt);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StationUltra_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::StationUltra_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::StationUltra_<ContainerAllocator>& v)
  {
    s << indent << "ultra_lt[]" << std::endl;
    for (size_t i = 0; i < v.ultra_lt.size(); ++i)
    {
      s << indent << "  ultra_lt[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.ultra_lt[i]);
    }
    s << indent << "ultra_rt[]" << std::endl;
    for (size_t i = 0; i < v.ultra_rt.size(); ++i)
    {
      s << indent << "  ultra_rt[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.ultra_rt[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_STATIONULTRA_H
