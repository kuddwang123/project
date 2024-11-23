// Generated by gencpp from file hj_interface/ErrorCodeStatistic.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_ERRORCODESTATISTIC_H
#define HJ_INTERFACE_MESSAGE_ERRORCODESTATISTIC_H

#include <ros/service_traits.h>


#include <hj_interface/ErrorCodeStatisticRequest.h>
#include <hj_interface/ErrorCodeStatisticResponse.h>


namespace hj_interface
{

struct ErrorCodeStatistic
{

typedef ErrorCodeStatisticRequest Request;
typedef ErrorCodeStatisticResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ErrorCodeStatistic
} // namespace hj_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hj_interface::ErrorCodeStatistic > {
  static const char* value()
  {
    return "23c5fe1389d0cf6a0a486460425cbc7b";
  }

  static const char* value(const ::hj_interface::ErrorCodeStatistic&) { return value(); }
};

template<>
struct DataType< ::hj_interface::ErrorCodeStatistic > {
  static const char* value()
  {
    return "hj_interface/ErrorCodeStatistic";
  }

  static const char* value(const ::hj_interface::ErrorCodeStatistic&) { return value(); }
};


// service_traits::MD5Sum< ::hj_interface::ErrorCodeStatisticRequest> should match
// service_traits::MD5Sum< ::hj_interface::ErrorCodeStatistic >
template<>
struct MD5Sum< ::hj_interface::ErrorCodeStatisticRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hj_interface::ErrorCodeStatistic >::value();
  }
  static const char* value(const ::hj_interface::ErrorCodeStatisticRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hj_interface::ErrorCodeStatisticRequest> should match
// service_traits::DataType< ::hj_interface::ErrorCodeStatistic >
template<>
struct DataType< ::hj_interface::ErrorCodeStatisticRequest>
{
  static const char* value()
  {
    return DataType< ::hj_interface::ErrorCodeStatistic >::value();
  }
  static const char* value(const ::hj_interface::ErrorCodeStatisticRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hj_interface::ErrorCodeStatisticResponse> should match
// service_traits::MD5Sum< ::hj_interface::ErrorCodeStatistic >
template<>
struct MD5Sum< ::hj_interface::ErrorCodeStatisticResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hj_interface::ErrorCodeStatistic >::value();
  }
  static const char* value(const ::hj_interface::ErrorCodeStatisticResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hj_interface::ErrorCodeStatisticResponse> should match
// service_traits::DataType< ::hj_interface::ErrorCodeStatistic >
template<>
struct DataType< ::hj_interface::ErrorCodeStatisticResponse>
{
  static const char* value()
  {
    return DataType< ::hj_interface::ErrorCodeStatistic >::value();
  }
  static const char* value(const ::hj_interface::ErrorCodeStatisticResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_ERRORCODESTATISTIC_H
