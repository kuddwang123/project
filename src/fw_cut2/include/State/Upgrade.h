#pragma once

#include "BaseState.h"
#include <condition_variable>
#include <mutex>
#include "Utils.h"
#include "Common.h"
#include "nlohmann/json.hpp"
#include "hj_interface/OtaUpgradeStatus.h"
#include "hj_interface/AirBagStatus.h"

namespace aiper_ota {

class Upgrade: public BaseState {

typedef struct {
    OtaModule module_;
    int ret_;
    std::string msg_;
}module_ota_ret;

public:
    Upgrade(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc);
    bool dowork(const boost::any& para) override;
    OtaModule getFailModule() const {return failModule_;}

private:
    bool upgradeAngo(const std::string& bin, const std::string& ver);
    bool upgradeLed(const std::string& bin, const std::string& ver);
    bool upgradeBase(const std::string& bin, const std::string& ver);
    bool upgradeSoc();
    void upgradeCb(const hj_interface::OtaUpgradeStatus::ConstPtr& msg);
    void airBagStatusCb(const hj_interface::AirBagStatus::ConstPtr& msg);

private:
    module_ota_ret curOtaRet_;
    ros::Publisher otaPub_;
    ros::Subscriber otaSub_;
    ros::Subscriber airbigSub_;
    uint8_t airBagStatus_;
    nlohmann::json cutRstJson_;
    OtaModule failModule_;
    std::condition_variable cond_;
    std::mutex mtx_;
};

typedef std::shared_ptr<Upgrade> UpgradePtr;
} //namespace aiper_ota