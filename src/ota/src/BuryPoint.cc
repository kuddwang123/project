#include "BuryPoint.h"
#include "Utils.h"
#include "hjlog.h"
#include "Common.h"
#include <chrono>
#include <fstream>

namespace aiper_ota {
const char* BuryPoint::file_ = "/userdata/.otaBuryPoint.json";

BuryPoint::BuryPoint():
    data_(nlohmann::json::object())
{

}

void BuryPoint::init(int mode, const std::string& cloudver)
{
    utils::DevInfo devinfo("/tmp/devInfo.json");
   
    data_["event"] = "otaResultEvent";
    data_["downloadStartTime"] = nullptr;
    data_["downloadEndTime"] = nullptr;
    data_["cloudVersion"] = cloudver;
    data_["downloadResult"] = nullptr;
    data_["upgradeStartTime"] = nullptr;
    data_["upgradeEndTime"] = nullptr;
    data_["failReason"] = 0;
    data_["externalMsg"] = "";
    data_["angoOta"] = nullptr;
    data_["status"] = nullptr;
    data_["type"] = mode;
    data_["fwVersion"] = devinfo.version();
}

bool BuryPoint::initFromFile()
{
    nlohmann::json loadjson;

    std::ifstream jsonFile(file_);
    if (!jsonFile.is_open()) {
        HJ_CST_TIME_ERROR(ota_logger, "cannot open file [%s]\n", file_);
        return false;
    }

    try {
        jsonFile >> loadjson;
    } catch (const nlohmann::json::parse_error& e) {
        HJ_CST_TIME_ERROR(ota_logger, "json parse error\n");
        return false;
    }

    jsonFile.close();

    HJ_CST_TIME_DEBUG(ota_logger, "load bury point:\n");
    HJ_CST_TIME_DEBUG(ota_logger, "%s\n", loadjson.dump(4).c_str());
    data_ = loadjson;

    return true;
}

void BuryPoint::startDownLoad()
{
    data_["downloadStartTime"] = getCurTimeMs();
}

void BuryPoint::endDownLoad(int type)
{
    data_["downloadEndTime"] = getCurTimeMs();
    if (type != BP_OK) {
        data_["failReason"] = type;
        data_["status"] = 0;
        data_["downloadResult"] = 0;
    } else {
        data_["downloadResult"] = 1;
    }
}

void BuryPoint::startUpgrade()
{
    data_["upgradeStartTime"] = getCurTimeMs();
}

void BuryPoint::endUpgrade(bool success, int failpart, int angoota, const std::string& failmsg)
{
    if (failpart >= BP_OK && failpart <= BP_SOC_OTA_FAIL) {
        data_["upgradeEndTime"] = getCurTimeMs();
    }
    
    if (!success) {
        data_["failReason"] = failpart;
        data_["status"] = 0;
        data_["externalMsg"] = failmsg;
    } else {
        data_["status"] = 1;
    }
    data_["angoOta"] = angoota;
    
    if (uploadFunc_) {
        uploadFunc_(data_.dump());
        utils::removeFile(file_);
    }
}

/*
void BuryPoint::triggerUpload()
{
    HJ_CST_TIME_DEBUG(ota_logger, "trigger bury point:\n");
    HJ_CST_TIME_DEBUG(ota_logger, "%s\n", data_.dump(4).c_str());

    hj_interface::BigdataUpload msg;
    msg.payload = data_.dump();
    buryPointPub_.publish(msg);
}
*/

long long BuryPoint::getCurTimeMs()
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return duration.count();
}

bool BuryPoint::saveBuryDataToFile(int angoota)
{
    data_["angoOta"] = angoota;
    std::ofstream jsonFile(file_, std::ios::trunc);
    if (!jsonFile.is_open()) {
        HJ_CST_TIME_DEBUG(ota_logger, "open file [%s] fail\n", file_);
        return false;
    }

    try {
        jsonFile << data_.dump();
    } catch (const std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "save json fail: %s\n", e.what());
        return false;
    }

    jsonFile.close();
    HJ_CST_TIME_DEBUG(ota_logger, "save json success:\n");
    HJ_CST_TIME_DEBUG(ota_logger, "%s\n", data_.dump().c_str());

    return true;
}

}