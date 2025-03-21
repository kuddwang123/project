// Generated by gencpp from file hj_interface/SlamNaviWorkResult.msg
// DO NOT EDIT!


#ifndef HJ_INTERFACE_MESSAGE_SLAMNAVIWORKRESULT_H
#define HJ_INTERFACE_MESSAGE_SLAMNAVIWORKRESULT_H

#include <ros/service_traits.h>


#include <hj_interface/SlamNaviWorkResultRequest.h>
#include <hj_interface/SlamNaviWorkResultResponse.h>


namespace hj_interface
{

struct SlamNaviWorkResult
{

typedef SlamNaviWorkResultRequest Request;
typedef SlamNaviWorkResultResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SlamNaviWorkResult
} // namespace hj_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hj_interface::SlamNaviWorkResult > {
  static const char* value()
  {
    return "8a0ceccc4349c854070b70848346678a";
  }

  static const char* value(const ::hj_interface::SlamNaviWorkResult&) { return value(); }
};

template<>
struct DataType< ::hj_interface::SlamNaviWorkResult > {
  static const char* value()
  {
    return "hj_interface/SlamNaviWorkResult";
  }

  static const char* value(const ::hj_interface::SlamNaviWorkResult&) { return value(); }
};


// service_traits::MD5Sum< ::hj_interface::SlamNaviWorkResultRequest> should match
// service_traits::MD5Sum< ::hj_interface::SlamNaviWorkResult >
template<>
struct MD5Sum< ::hj_interface::SlamNaviWorkResultRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hj_interface::SlamNaviWorkResult >::value();
  }
  static const char* value(const ::hj_interface::SlamNaviWorkResultRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hj_interface::SlamNaviWorkResultRequest> should match
// service_traits::DataType< ::hj_interface::SlamNaviWorkResult >
template<>
struct DataType< ::hj_interface::SlamNaviWorkResultRequest>
{
  static const char* value()
  {
    return DataType< ::hj_interface::SlamNaviWorkResult >::value();
  }
  static const char* value(const ::hj_interface::SlamNaviWorkResultRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hj_interface::SlamNaviWorkResultResponse> should match
// service_traits::MD5Sum< ::hj_interface::SlamNaviWorkResult >
template<>
struct MD5Sum< ::hj_interface::SlamNaviWorkResultResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hj_interface::SlamNaviWorkResult >::value();
  }
  static const char* value(const ::hj_interface::SlamNaviWorkResultResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hj_interface::SlamNaviWorkResultResponse> should match
// service_traits::DataType< ::hj_interface::SlamNaviWorkResult >
template<>
struct DataType< ::hj_interface::SlamNaviWorkResultResponse>
{
  static const char* value()
  {
    return DataType< ::hj_interface::SlamNaviWorkResult >::value();
  }
  static const char* value(const ::hj_interface::SlamNaviWorkResultResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HJ_INTERFACE_MESSAGE_SLAMNAVIWORKRESULT_H
