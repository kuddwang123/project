#include "State/Upgrade.h"
#include "hj_interface/OtaUpgradeData.h"
#include "hjlog.h"
#include <string>
#include <thread>

namespace aiper_ota {

Upgrade::Upgrade(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc):
    BaseState(n, otaRptFunc),
    airBagStatus_(0),
    cutRstJson_(nlohmann::json::object()),
    failModule_(none)
{
    otaPub_ = n_.advertise<hj_interface::OtaUpgradeData>("/system/eventNotify", 1);
    otaSub_ = n_.subscribe("/system/eventStatusRep", 1, &Upgrade::upgradeCb, this); 
    airbigSub_ = n_.subscribe("/AirBagStatusNew", 1, &Upgrade::airBagStatusCb, this);
}

void Upgrade::airBagStatusCb(const hj_interface::AirBagStatus::ConstPtr& msg)
{
    airBagStatus_ = msg->status;
}

bool Upgrade::dowork(const boost::any& para)
{
    failModule_ = none;
    status_ = WORKING;
    std::string info = boost::any_cast<std::string>(para);
    nlohmann::json baseJson(nlohmann::json::object());
    nlohmann::json ledJson(nlohmann::json::object());
    nlohmann::json angoJson(nlohmann::json::object());

    try {
        nlohmann::json cutRstJson = nlohmann::json::parse(info);
        angoJson = cutRstJson["mcu"]["ango"];
        baseJson = cutRstJson["mcu"]["baseboard"];
        ledJson = cutRstJson["mcu"]["ledboard"];
    } catch (const std::exception& e) {
        fprintf(stderr, "json parse error\n");
        return false;
    }
    
    if(!angoJson.contains("path") || !angoJson.contains("version")) {
        fprintf(stderr, "ango json parse error\n");
        return false;
    }

    if(!baseJson.contains("path") || !baseJson.contains("version")) {
        fprintf(stderr, "base json parse error\n");
        return false;
    }

    if(!ledJson.contains("path") || !ledJson.contains("version")) {
        fprintf(stderr, "led json parse error\n");
        return false;
    }

    int abcnt = 0;
    while (airBagStatus_ != 0) {
        fprintf(stdout, "air bag release not complete:%d\n", airBagStatus_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++abcnt;
        if (abcnt >= 50) {
            fprintf(stderr, "air bag release time out\n");
            return false;
        }
    }
    fprintf(stdout, "air bag check done:%d\n", airBagStatus_);
    
    if (!upgradeAngo(angoJson["path"], angoJson["version"])) {
        fprintf(stderr, "ota ango fail!\n");
        //failModule_ = ango;
        //return false;
    } else {
        fprintf(stdout, "ota ango success to [%s]!\n", angoJson["version"].get<std::string>().c_str());
    }

    if (!upgradeLed(ledJson["path"], ledJson["version"])) {
        fprintf(stderr, "ota led fail!\n");
        failModule_ = ledboard;
        return false;
    }
    fprintf(stdout, "ota led success to [%s]!\n", ledJson["version"].get<std::string>().c_str());

    if (!upgradeBase(baseJson["path"], baseJson["version"])) {
        fprintf(stderr, "ota base fail!\n");
        failModule_ = baseboard;
        return false;
    }  
    fprintf(stdout, "ota base success to [%s]!\n", baseJson["version"].get<std::string>().c_str());
    
    if (!upgradeSoc()) {
        fprintf(stderr, "ota soc fail!\n");
        failModule_ = soc;
        return false;
    }
    fprintf(stdout, "ota soc success!\n");

    return true;
}

bool Upgrade::upgradeAngo(const std::string& bin, const std::string& ver)
{
    nlohmann::json mcu;
    hj_interface::OtaUpgradeData msg;
    msg.todo = "UpdateOTA";
    msg.stage = 1;
    msg.module = ango;
    msg.data.addr = bin;
    msg.data.ver = ver;

    int retry = 2;
    bool rst = false;
    fprintf(stdout, "ango ota...\n");

    while (--retry >= 0) {
        otaPub_.publish(msg);

        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
            return curOtaRet_.module_ == ango;
        })) {
            fprintf(stderr, "ango ota timeout!\n");
            continue;
        } 

        if (curOtaRet_.ret_ != 0) {
            fprintf(stderr, "ango ota fail: %s!\n", curOtaRet_.msg_.c_str());
            curOtaRet_.module_ = none;
            continue;
        } else {
            rst = true;
            break;
        }
    }
    return rst;
}

bool Upgrade::upgradeLed(const std::string& bin, const std::string& ver)
{
    nlohmann::json mcu;
    hj_interface::OtaUpgradeData msg;
    msg.todo = "UpdateOTA";
    msg.stage = 1;
    msg.module = ledboard;
    msg.data.addr = bin;
    msg.data.ver = ver;

    int retry = 2;
    bool rst = false;
    fprintf(stdout, "led ota...\n");

    while (--retry >= 0) {
        otaPub_.publish(msg);

        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
            return curOtaRet_.module_ == ledboard;
        })) {
            fprintf(stderr, "led ota timeout!\n");
            continue;
        } 

        if (curOtaRet_.ret_ != 0) {
            fprintf(stderr, "led ota fail: %s!\n", curOtaRet_.msg_.c_str());
            curOtaRet_.module_ = none;
            continue;
        } else {
            rst = true;
            break;
        }
    }
    return rst;
}

bool Upgrade::upgradeBase(const std::string& bin, const std::string& ver)
{
    nlohmann::json mcu;
    hj_interface::OtaUpgradeData msg;
    msg.todo = "UpdateOTA";
    msg.stage = 1;
    msg.module = baseboard;
    msg.data.addr = bin;
    msg.data.ver = ver;

    int retry = 2;
    bool rst = false;
    fprintf(stdout, "base ota...\n");

    while (--retry >= 0) {
        otaPub_.publish(msg);

        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
            return curOtaRet_.module_ == baseboard;
        })) {
            fprintf(stderr, "baseboard ota timeout!\n");
            continue;
        } 

        if (curOtaRet_.ret_ != 0) {
            fprintf(stderr, "baseboard ota fail: %s!\n", curOtaRet_.msg_.c_str());
            curOtaRet_.module_ = none;
            continue;
        } else {
            rst = true;
            break;
        }
    }
    return rst;
}

bool Upgrade::upgradeSoc()
{
    hj_interface::OtaUpgradeData msg;
    msg.todo = "UpdateOTA";
    msg.stage = 1;
    msg.module = soc;
    otaPub_.publish(msg);

    std::unique_lock<std::mutex> lc(mtx_);
    if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
        return curOtaRet_.module_ == soc;
    })) {
        fprintf(stderr, "soc ota timeout!\n");
        return false;
    } 

    if (curOtaRet_.ret_ != 0) {
        fprintf(stderr, "soc ota fail: %s!\n", curOtaRet_.msg_.c_str());
        return false;
    }

    return true;
}

void Upgrade::upgradeCb(const hj_interface::OtaUpgradeStatus::ConstPtr& msg)
{
    if (status_ != WORKING) {
        fprintf(stdout, "upgrade not working\n");
        return;
    }
/*
    fprintf(stdout, "[ota resp] todo = %s\n", msg->todo.c_str());
    fprintf(stdout, "[ota resp] stage = %d\n", msg->stage);
    fprintf(stdout, "[ota resp] module = %d\n", msg->module);
    fprintf(stdout, "[ota resp] ret = %d\n", msg->ret);
    fprintf(stdout, "[ota resp] msg = %s\n", msg->msg.c_str());
*/
    std::unique_lock<std::mutex> lc(mtx_);
    curOtaRet_.module_ = static_cast<OtaModule>(msg->module);
    curOtaRet_.ret_ = msg->ret;
    curOtaRet_.msg_ = msg->msg;
    cond_.notify_one();
}

}//namespace aiper_ota