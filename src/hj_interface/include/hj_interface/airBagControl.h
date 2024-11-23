// Generated by gencpp from file hj_interface/airBagControl.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_AIRBAGCONTROL_H
#define HJ_INTERFACE_MESSAGE_AIRBAGCONTROL_H

#include <ros/service_traits.h>


#include <hj_interface/airBagControlRequest.h>
#include <hj_interface/airBagControlResponse.h>


namespace hj_interface
{

struct airBagControl
{

typedef airBagControlRequest Request;
typedef airBagControlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct airBagControl
} // namespace hj_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hj_interface::airBagControl > {
  static const char* value()
  {
    return "b6a67de01f896e717e870c3512f192ef";
  }

  static const char* value(const ::hj_interface::airBagControl&) { return value(); }
};

template<>
struct DataType< ::hj_interface::airBagControl > {
  static const char* value()
  {
    return "hj_interface/airBagControl";
  }

  static const char* value(const ::hj_interface::airBagControl&) { return value(); }
};


// service_traits::MD5Sum< ::hj_interface::airBagControlRequest> should match
// service_traits::MD5Sum< ::hj_interface::airBagControl >
template<>
struct MD5Sum< ::hj_interface::airBagControlRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hj_interface::airBagControl >::value();
  }
  static const char* value(const ::hj_interface::airBagControlRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hj_interface::airBagControlRequest> should match
// service_traits::DataType< ::hj_interface::airBagControl >
template<>
struct DataType< ::hj_interface::airBagControlRequest>
{
  static const char* value()
  {
    return DataType< ::hj_interface::airBagControl >::value();
  }
  static const char* value(const ::hj_interface::airBagControlRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hj_interface::airBagControlResponse> should match
// service_traits::MD5Sum< ::hj_interface::airBagControl >
template<>
struct MD5Sum< ::hj_interface::airBagControlResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hj_interface::airBagControl >::value();
  }
  static const char* value(const ::hj_interface::airBagControlResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hj_interface::airBagControlResponse> should match
// service_traits::DataType< ::hj_interface::airBagControl >
template<>
struct DataType< ::hj_interface::airBagControlResponse>
{
  static const char* value()
  {
    return DataType< ::hj_interface::airBagControl >::value();
  }
  static const char* value(const ::hj_interface::airBagControlResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_AIRBAGCONTROL_H
