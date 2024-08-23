#pragma  once

#include <boost/function.hpp>
#include <string>
namespace collect_node_iot {
enum {
    BLUETOOTH = 0x01,
    IOT_SHADOW = 0x02,
    IOT_TOPIC = 0x04,
    IOT_CLOUD_TOPIC = 0x06,
    CLOUD = 0x08,
    BIGDATA = 0x10,
    WIFI = 0x20
};

typedef boost::function<void()> ConnDisConCb;
typedef boost::function<void(bool)> ConStaChangeCb;
typedef boost::function<void(int, std::string)> ConnFailCb;
typedef boost::function<void(int)> socketUpdateFunc;
typedef boost::function<void(const char*, size_t)> SockReadCompleteCb;
}