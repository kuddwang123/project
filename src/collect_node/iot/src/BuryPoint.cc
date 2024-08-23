
#include "BuryPoint.h"
#include "Utils.h"
#include "big_data.h"
#include "log.h"
#include "hj_interface/AppMsg.h"
#include <fstream>
#include <ctime>
#include <chrono>

namespace collect_node_iot {

BaseBuryPoint::BaseBuryPoint(const std::string& evtid):
    evtid_(evtid)
{
    utils::DevInfo devInfo("/tmp/devInfo.json");
    if (devInfo.openSucc()) {
        firmVer_ = devInfo.version();
    }

    initJsonDoc();
}

void BaseBuryPoint::sendBigData()
{
    std::string senddata = utils::documentToString(jsondoc_);
    HJ_INFO("big data send:\n%s\n", senddata.c_str());
    big_data::InsertBigdata(senddata);
}

void BaseBuryPoint::initJsonDoc()
{
    if (jsondoc_.IsObject()) {
        jsondoc_.RemoveAllMembers();
    }
    
    jsondoc_.SetObject();
    jsondoc_.AddMember("event", rapidjson::Value(evtid_.data(), jsondoc_.GetAllocator()).Move(),
                jsondoc_.GetAllocator());
    jsondoc_.AddMember("fwVersion", rapidjson::Value(firmVer_.data(), jsondoc_.GetAllocator()).Move(),
                jsondoc_.GetAllocator());
}

BaseBuryPointFactory& BaseBuryPointFactory::instance()
{
    static BaseBuryPointFactory instance;
    return instance;
}

const BuryPointPtr BaseBuryPointFactory::createBuryPoint(BuryPointType type)
{
    std::unique_lock<std::mutex> lc(mtx_);
    BuryPointPtr ptr;
    switch (type) {
        case kNET_CONFIG:
            ptr = std::make_shared<NetConfigBuryPoint>("netConnectionEvent");
            buryPointData_.push_back(ptr);
            break;
        
        default:
            HJ_ERROR("unknown type:%d\n", type);
    }

    return ptr;
}

const BuryPointPtr BaseBuryPointFactory::getBuryPoint(BuryPointType type)
{
    std::unique_lock<std::mutex> lc(mtx_);
    for (auto const& data: buryPointData_) {
        if (data->getType() == type) {
            return data;
        }
    }

    return nullptr;
}

NetConfigBuryPoint::NetConfigBuryPoint(const std::string& id):
    BaseBuryPoint(id),
    system_start_time_(0),
    start_ts_(0),
    route_(-1)
{
    type_ = BaseBuryPointFactory::kNET_CONFIG;
}

time_t NetConfigBuryPoint::getSystemUptimeSec()
{
    std::ifstream uptimeFile("/proc/uptime");
    if (!uptimeFile.is_open()) {
        HJ_ERROR("Failed to open /proc/uptime\n");
        return 0;
    }

    time_t uptimeSeconds;
    uptimeFile >> uptimeSeconds;

    return uptimeSeconds;
}

void NetConfigBuryPoint::recordStartTime()
{
    system_start_time_ = getSystemUptimeSec();
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    start_ts_ = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void NetConfigBuryPoint::triggerBigDataRpt(bool result, int code)
{
    if (start_ts_ == 0) {
        return;
    }

    time_t now = getSystemUptimeSec();
    uint8_t route = 255;
    if (route_ == hj_interface::AppMsg::BLUETOOTH) {
        route = 0;
    } else if (route_ == hj_interface::AppMsg::AP) {
        route = 1;
    } else {
        route = 0;
    }

    jsondoc_.AddMember("status", result ? 1 : 0, jsondoc_.GetAllocator());
    jsondoc_.AddMember("route", route, jsondoc_.GetAllocator());
    jsondoc_.AddMember("failReason", code, jsondoc_.GetAllocator());
    jsondoc_.AddMember("duration", now-system_start_time_, jsondoc_.GetAllocator());
    jsondoc_.AddMember("startTime", start_ts_, jsondoc_.GetAllocator());

    BaseBuryPoint::sendBigData();
    BaseBuryPoint::initJsonDoc();

    system_start_time_ = 0;
    start_ts_ = 0;
    route_ = -1;
}

void NetConfigBuryPoint::triggerBigDataRptWithMsg(bool result, int code, const std::string msg)
{
    if (start_ts_ == 0) {
        return;
    }

    time_t now = getSystemUptimeSec();
    uint8_t route = 255;
    if (route_ == hj_interface::AppMsg::BLUETOOTH) {
        route = 0;
    } else if (route_ == hj_interface::AppMsg::AP) {
        route = 1;
    } else {
        route = 0;
    }

    jsondoc_.AddMember("status", result ? 1 : 0, jsondoc_.GetAllocator());
    jsondoc_.AddMember("route", route, jsondoc_.GetAllocator());
    jsondoc_.AddMember("failReason", code, jsondoc_.GetAllocator());
    jsondoc_.AddMember("message", rapidjson::Value(msg.data(), jsondoc_.GetAllocator()).Move(), jsondoc_.GetAllocator());
    jsondoc_.AddMember("duration", now-system_start_time_, jsondoc_.GetAllocator());
    jsondoc_.AddMember("startTime", start_ts_, jsondoc_.GetAllocator());

    BaseBuryPoint::sendBigData();
    BaseBuryPoint::initJsonDoc();

    system_start_time_ = 0;
    start_ts_ = 0;
    route_ = -1;
}


} // namespace collect_node_iot