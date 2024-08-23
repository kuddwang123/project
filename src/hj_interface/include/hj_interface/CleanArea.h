// Generated by gencpp from file hj_interface/CleanArea.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_CLEANAREA_H
#define HJ_INTERFACE_MESSAGE_CLEANAREA_H


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
struct CleanArea_
{
  typedef CleanArea_<ContainerAllocator> Type;

  CleanArea_()
    : clean_area()
    , pool_bottom(0)
    , pool_wall(0)
    , water_line(0)
    , water_surface(0)
    , step(0)  {
    }
  CleanArea_(const ContainerAllocator& _alloc)
    : clean_area(_alloc)
    , pool_bottom(0)
    , pool_wall(0)
    , water_line(0)
    , water_surface(0)
    , step(0)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _clean_area_type;
  _clean_area_type clean_area;

   typedef int32_t _pool_bottom_type;
  _pool_bottom_type pool_bottom;

   typedef int32_t _pool_wall_type;
  _pool_wall_type pool_wall;

   typedef int32_t _water_line_type;
  _water_line_type water_line;

   typedef int32_t _water_surface_type;
  _water_surface_type water_surface;

   typedef int32_t _step_type;
  _step_type step;





  typedef boost::shared_ptr< ::hj_interface::CleanArea_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::CleanArea_<ContainerAllocator> const> ConstPtr;

}; // struct CleanArea_

typedef ::hj_interface::CleanArea_<std::allocator<void> > CleanArea;

typedef boost::shared_ptr< ::hj_interface::CleanArea > CleanAreaPtr;
typedef boost::shared_ptr< ::hj_interface::CleanArea const> CleanAreaConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::CleanArea_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::CleanArea_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::CleanArea_<ContainerAllocator1> & lhs, const ::hj_interface::CleanArea_<ContainerAllocator2> & rhs)
{
  return lhs.clean_area == rhs.clean_area &&
    lhs.pool_bottom == rhs.pool_bottom &&
    lhs.pool_wall == rhs.pool_wall &&
    lhs.water_line == rhs.water_line &&
    lhs.water_surface == rhs.water_surface &&
    lhs.step == rhs.step;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::CleanArea_<ContainerAllocator1> & lhs, const ::hj_interface::CleanArea_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::CleanArea_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::CleanArea_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::CleanArea_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::CleanArea_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::CleanArea_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::CleanArea_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::CleanArea_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d29c0130f06a580f71148ca2b7b348ec";
  }

  static const char* value(const ::hj_interface::CleanArea_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd29c0130f06a580fULL;
  static const uint64_t static_value2 = 0x71148ca2b7b348ecULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::CleanArea_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/CleanArea";
  }

  static const char* value(const ::hj_interface::CleanArea_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::CleanArea_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] clean_area  # 1-池底 2-池壁 3-水线 4-水面 5-台阶 FIFO 清扫顺序\n"
"int32 pool_bottom   # 池底次数\n"
"int32 pool_wall     # 池壁次数\n"
"int32 water_line    # 水线次数\n"
"int32 water_surface # 水面次数\n"
"int32 step          # 台阶次数\n"
;
  }

  static const char* value(const ::hj_interface::CleanArea_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::CleanArea_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.clean_area);
      stream.next(m.pool_bottom);
      stream.next(m.pool_wall);
      stream.next(m.water_line);
      stream.next(m.water_surface);
      stream.next(m.step);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CleanArea_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::CleanArea_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::CleanArea_<ContainerAllocator>& v)
  {
    s << indent << "clean_area[]" << std::endl;
    for (size_t i = 0; i < v.clean_area.size(); ++i)
    {
      s << indent << "  clean_area[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.clean_area[i]);
    }
    s << indent << "pool_bottom: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pool_bottom);
    s << indent << "pool_wall: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pool_wall);
    s << indent << "water_line: ";
    Printer<int32_t>::stream(s, indent + "  ", v.water_line);
    s << indent << "water_surface: ";
    Printer<int32_t>::stream(s, indent + "  ", v.water_surface);
    s << indent << "step: ";
    Printer<int32_t>::stream(s, indent + "  ", v.step);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_CLEANAREA_H
