// Generated by gencpp from file edg_data_logger/Enable.msg
// DO NOT EDIT!


#ifndef EDG_DATA_LOGGER_MESSAGE_ENABLE_H
#define EDG_DATA_LOGGER_MESSAGE_ENABLE_H

#include <ros/service_traits.h>


#include <edg_data_logger/EnableRequest.h>
#include <edg_data_logger/EnableResponse.h>


namespace edg_data_logger
{

struct Enable
{

typedef EnableRequest Request;
typedef EnableResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Enable
} // namespace edg_data_logger


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::edg_data_logger::Enable > {
  static const char* value()
  {
    return "f3f19e69803c97782b2fdad054f37b22";
  }

  static const char* value(const ::edg_data_logger::Enable&) { return value(); }
};

template<>
struct DataType< ::edg_data_logger::Enable > {
  static const char* value()
  {
    return "edg_data_logger/Enable";
  }

  static const char* value(const ::edg_data_logger::Enable&) { return value(); }
};


// service_traits::MD5Sum< ::edg_data_logger::EnableRequest> should match
// service_traits::MD5Sum< ::edg_data_logger::Enable >
template<>
struct MD5Sum< ::edg_data_logger::EnableRequest>
{
  static const char* value()
  {
    return MD5Sum< ::edg_data_logger::Enable >::value();
  }
  static const char* value(const ::edg_data_logger::EnableRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::edg_data_logger::EnableRequest> should match
// service_traits::DataType< ::edg_data_logger::Enable >
template<>
struct DataType< ::edg_data_logger::EnableRequest>
{
  static const char* value()
  {
    return DataType< ::edg_data_logger::Enable >::value();
  }
  static const char* value(const ::edg_data_logger::EnableRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::edg_data_logger::EnableResponse> should match
// service_traits::MD5Sum< ::edg_data_logger::Enable >
template<>
struct MD5Sum< ::edg_data_logger::EnableResponse>
{
  static const char* value()
  {
    return MD5Sum< ::edg_data_logger::Enable >::value();
  }
  static const char* value(const ::edg_data_logger::EnableResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::edg_data_logger::EnableResponse> should match
// service_traits::DataType< ::edg_data_logger::Enable >
template<>
struct DataType< ::edg_data_logger::EnableResponse>
{
  static const char* value()
  {
    return DataType< ::edg_data_logger::Enable >::value();
  }
  static const char* value(const ::edg_data_logger::EnableResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // EDG_DATA_LOGGER_MESSAGE_ENABLE_H
