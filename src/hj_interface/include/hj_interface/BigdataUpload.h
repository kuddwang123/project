// Generated by gencpp from file hj_interface/BigdataUpload.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_BIGDATAUPLOAD_H
#define HJ_INTERFACE_MESSAGE_BIGDATAUPLOAD_H


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
struct BigdataUpload_
{
  typedef BigdataUpload_<ContainerAllocator> Type;

  BigdataUpload_()
    : payload()
    , upload_file()
    , type(0)  {
    }
  BigdataUpload_(const ContainerAllocator& _alloc)
    : payload(_alloc)
    , upload_file(_alloc)
    , type(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _payload_type;
  _payload_type payload;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _upload_file_type;
  _upload_file_type upload_file;

   typedef uint8_t _type_type;
  _type_type type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(kBigdataImmediate)
  #undef kBigdataImmediate
#endif
#if defined(_WIN32) && defined(kBigdataPack)
  #undef kBigdataPack
#endif
#if defined(_WIN32) && defined(kBigdataDelete)
  #undef kBigdataDelete
#endif

  enum {
    kBigdataImmediate = 0u,
    kBigdataPack = 1u,
    kBigdataDelete = 2u,
  };


  typedef boost::shared_ptr< ::hj_interface::BigdataUpload_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hj_interface::BigdataUpload_<ContainerAllocator> const> ConstPtr;

}; // struct BigdataUpload_

typedef ::hj_interface::BigdataUpload_<std::allocator<void> > BigdataUpload;

typedef boost::shared_ptr< ::hj_interface::BigdataUpload > BigdataUploadPtr;
typedef boost::shared_ptr< ::hj_interface::BigdataUpload const> BigdataUploadConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hj_interface::BigdataUpload_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hj_interface::BigdataUpload_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hj_interface::BigdataUpload_<ContainerAllocator1> & lhs, const ::hj_interface::BigdataUpload_<ContainerAllocator2> & rhs)
{
  return lhs.payload == rhs.payload &&
    lhs.upload_file == rhs.upload_file &&
    lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hj_interface::BigdataUpload_<ContainerAllocator1> & lhs, const ::hj_interface::BigdataUpload_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hj_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hj_interface::BigdataUpload_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hj_interface::BigdataUpload_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::BigdataUpload_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hj_interface::BigdataUpload_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::BigdataUpload_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hj_interface::BigdataUpload_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hj_interface::BigdataUpload_<ContainerAllocator> >
{
  static const char* value()
  {
    return "11d9f39ce9710c7ecd48b2112ded9f2a";
  }

  static const char* value(const ::hj_interface::BigdataUpload_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x11d9f39ce9710c7eULL;
  static const uint64_t static_value2 = 0xcd48b2112ded9f2aULL;
};

template<class ContainerAllocator>
struct DataType< ::hj_interface::BigdataUpload_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hj_interface/BigdataUpload";
  }

  static const char* value(const ::hj_interface::BigdataUpload_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hj_interface::BigdataUpload_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string payload # 大数据已组好的内容\n"
"string upload_file #带压缩包上传时，压缩包路径\n"
"uint8  kBigdataImmediate = 0 # 立刻发送，当前版本策略必带\n"
"uint8  kBigdataPack = 1 #需要带压缩包时(upload_file)，带上这个参数\n"
"uint8  kBigdataDelete = 2 # 默认压缩包上传成功不删除，带上这个参数会帮你删除\n"
"uint8  type #上述参数或关系，如：kBigdataImmediate|kBigdataPack|kBigdataDelete\n"
;
  }

  static const char* value(const ::hj_interface::BigdataUpload_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hj_interface::BigdataUpload_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.payload);
      stream.next(m.upload_file);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BigdataUpload_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hj_interface::BigdataUpload_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hj_interface::BigdataUpload_<ContainerAllocator>& v)
  {
    s << indent << "payload: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.payload);
    s << indent << "upload_file: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.upload_file);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_BIGDATAUPLOAD_H