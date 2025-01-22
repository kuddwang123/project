#pragma once
#include <ros/ros.h>
#include "nlohmann/json.hpp"
#include "Common.h"
#include <string>

namespace aiper_ota {

class BuryPoint {
public:
    BuryPoint();
    ~BuryPoint() {};

    void init(int mode, const std::string& cloudver);
    bool initFromFile();
    void startDownLoad();
    void endDownLoad(int type);
    void startUpgrade();
    void endUpgrade(bool success, int failpart, int angoota, const std::string& failmsg);
    //void triggerUpload();
    bool saveBuryDataToFile(int angoota);
    void setTriggetCb(const buryPointUploadFunc& func) {uploadFunc_ = func;}

private:
    long long getCurTimeMs();
    
private:
    static const char* file_;
    buryPointUploadFunc uploadFunc_;

private:
    nlohmann::json data_;
/*    
    std::string dlVer_;
    int mode_;
    int angoOta_;
    time_t startDlTime_;
    time_t stopDlTime_;
    time_t startUgTime_;
    time_t stopUgTime_;
*/
}; 

}