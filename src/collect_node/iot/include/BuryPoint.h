#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

namespace collect_node_iot {
class BaseBuryPoint;
class NetConfigBuryPoint;
typedef std::shared_ptr<BaseBuryPoint> BuryPointPtr;
typedef std::shared_ptr<NetConfigBuryPoint> NetCfgBuryPointPtr;

class BaseBuryPointFactory {
public:
    enum BuryPointType{
        kNET_CONFIG,
    };

public:
    static BaseBuryPointFactory& instance();
    BaseBuryPointFactory(const BaseBuryPointFactory& other) = delete;
    BaseBuryPointFactory& operator=(const BaseBuryPointFactory& other) = delete;
    const BuryPointPtr createBuryPoint(BuryPointType type);
    const BuryPointPtr getBuryPoint(BuryPointType type);

private:
    BaseBuryPointFactory() {};

private:
    std::mutex mtx_;
    std::vector<BuryPointPtr> buryPointData_;
};

class BaseBuryPoint {
public:
    BaseBuryPoint(const std::string& evtid);
    BaseBuryPointFactory::BuryPointType getType() const {return type_;};
    virtual ~BaseBuryPoint() {}

protected:
    void initJsonDoc();
    void sendBigData();

protected:
    std::string evtid_;
    std::string firmVer_;
    rapidjson::Document jsondoc_;
    BaseBuryPointFactory::BuryPointType type_;
};

class NetConfigBuryPoint: public BaseBuryPoint {
public:
    enum {
        kCONFIG_OK = 0,
        kJSON_PARSE_FAIL = 1000,
        kINTERNAL_INIT_FAIL,
        kINTERNAL_POST_FAIL,
        kNETCONFIG_JSON_VALUE_INVALID,
        KWIFI_CONFIG_SSID_NOTFOUD,
        kWiFI_CONFIG_PASSWD_ERROR,
        kWIFI_CONFIG_IP_ALLOC_FAIL,
        kWIFI_CONFIG_WIFI_WEAK,
        kWIFI_CONFIG_TIMEOUT,
        kWIFI_CONFIG_FAIL_UNKNOWN,
        kAWS_CERTFILE_FAIL,
        kIOT_CONNECT_FAIL
    };

public:
    NetConfigBuryPoint(const std::string& id);
    void recordStartTime();
    void setRoute(uint8_t route) {route_ = route;};
    void triggerBigDataRpt(bool result, int code = 0);
    void triggerBigDataRptWithMsg(bool result, int code, const std::string msg);
private:
    time_t getSystemUptimeSec();

private:
    time_t system_start_time_;
    time_t start_ts_;
    int route_;
};

} //namespase collect_node_iot