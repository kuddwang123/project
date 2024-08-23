// Generated by gencpp from file hj_interface/Bat.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_BAT_H
#define HJ_INTERFACE_MESSAGE_BAT_H


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
struct Bat_
{
  typedef Bat_<ContainerAllocator> Type;

  Bat_()
    : power(0)
    , temp1(0)
    , temp2(0)
    , temp3(0)
    , bat_vol(0)
    , bat_disch_cur(0)
    , ch_vol(0)
    , charger_ch_cur(0)
    , bat_cycle_times(0)
    , bat_health_left(0)  {
    }
  Bat_(const ContainerAllocator& _alloc)
    : power(0)
    , temp1(0)
    , temp2(0)
    , temp3(0)
    , bat_vol(0)
    , bat_disch_cur(0)
    , ch_vol(0)
    , charger_ch_cur(0)
    , bat_cycle_times(0)
    , bat_health_left(0)  {
  (void)_alloc;
    }



   typedef uint8_t _power_type;
  _power_type power;

   typedef int8_t _temp1_type;
  _temp1_type temp1;

   typedef int8_t _temp2_type;
  _temp2_type temp2;

   typedef int8_t _temp3_type;
  _temp3_type temp3;

   typedef uint16_t _bat_vol_type;
  _bat_vol_type bat_vol;

   typedef int16_t _bat_disch_cur_type;
  _bat_disch_cur_type bat_disch_cur;

   typedef uint16_t _ch_vol_type;
  _ch_vol_type ch_vol;

   typedef int16_t _charger_ch_cur_type;
  _charger_ch_cur_type charger_ch_cur;

   typedef uint16_t _bat_cycle_times_type;
  _bat_cycle_times_type bat_cycle_times;

   typedef uint8_t _bat_health_left_type;
  _bat_health_left_type bat_health_left;





  typedef boost::shared_ptr< ::hj_interface::Bat_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::Bat_<ContainerAllocator> const> ConstPtr;

}; // struct Bat_

typedef ::hj_interface::Bat_<std::allocator<void> > Bat;

typedef boost::shared_ptr< ::hj_interface::Bat > BatPtr;
typedef boost::shared_ptr< ::hj_interface::Bat const> BatConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::Bat_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::Bat_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::Bat_<ContainerAllocator1> & lhs, const ::hj_interface::Bat_<ContainerAllocator2> & rhs)
{
  return lhs.power == rhs.power &&
    lhs.temp1 == rhs.temp1 &&
    lhs.temp2 == rhs.temp2 &&
    lhs.temp3 == rhs.temp3 &&
    lhs.bat_vol == rhs.bat_vol &&
    lhs.bat_disch_cur == rhs.bat_disch_cur &&
    lhs.ch_vol == rhs.ch_vol &&
    lhs.charger_ch_cur == rhs.charger_ch_cur &&
    lhs.bat_cycle_times == rhs.bat_cycle_times &&
    lhs.bat_health_left == rhs.bat_health_left;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::Bat_<ContainerAllocator1> & lhs, const ::hj_interface::Bat_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::Bat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::Bat_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::Bat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::Bat_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::Bat_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::Bat_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::Bat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "67beab113d23636ee0599f8ce7cdb781";
  }

  static const char* value(const ::hj_interface::Bat_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x67beab113d23636eULL;
  static const uint64_t static_value2 = 0xe0599f8ce7cdb781ULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::Bat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/Bat";
  }

  static const char* value(const ::hj_interface::Bat_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::Bat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8   power              #电量%\n"
"int8    temp1              #温度1 \n"
"int8    temp2              #温度2\n"
"int8    temp3              #温度3\n"
"uint16  bat_vol            #电池电压mV\n"
"int16   bat_disch_cur      #电池放电电流mA\n"
"uint16  ch_vol             #电池充电电压mV\n"
"int16   charger_ch_cur     #充电端充电电流mA\n"
"uint16  bat_cycle_times    #电池循环次数\n"
"uint8   bat_health_left    #电池健康剩余容量\n"
;
  }

  static const char* value(const ::hj_interface::Bat_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::Bat_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.power);
      stream.next(m.temp1);
      stream.next(m.temp2);
      stream.next(m.temp3);
      stream.next(m.bat_vol);
      stream.next(m.bat_disch_cur);
      stream.next(m.ch_vol);
      stream.next(m.charger_ch_cur);
      stream.next(m.bat_cycle_times);
      stream.next(m.bat_health_left);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Bat_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::Bat_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::Bat_<ContainerAllocator>& v)
  {
    s << indent << "power: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.power);
    s << indent << "temp1: ";
    Printer<int8_t>::stream(s, indent + "  ", v.temp1);
    s << indent << "temp2: ";
    Printer<int8_t>::stream(s, indent + "  ", v.temp2);
    s << indent << "temp3: ";
    Printer<int8_t>::stream(s, indent + "  ", v.temp3);
    s << indent << "bat_vol: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.bat_vol);
    s << indent << "bat_disch_cur: ";
    Printer<int16_t>::stream(s, indent + "  ", v.bat_disch_cur);
    s << indent << "ch_vol: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.ch_vol);
    s << indent << "charger_ch_cur: ";
    Printer<int16_t>::stream(s, indent + "  ", v.charger_ch_cur);
    s << indent << "bat_cycle_times: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.bat_cycle_times);
    s << indent << "bat_health_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bat_health_left);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_BAT_H
