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
    mcuVerJson_(nlohmann::json::object()),
    failModule_(none)
{
    otaPub_ = n_.advertise<hj_interface::OtaUpgradeData>("/system/eventNotify", 1);
    otaSub_ = n_.subscribe("/system/eventStatusRep", 1, &Upgrade::upgradeCb, this); 
    airbigSub_ = n_.subscribe("/AirBagStatusNew", 1, &Upgrade::airBagStatusCb, this);
    mcuVerSub_ = n_.subscribe("mcuVer_chatter", 1, &Upgrade::mcuVerCallBack, this);
}

void Upgrade::airBagStatusCb(const hj_interface::AirBagStatus::ConstPtr& msg)
{
    airBagStatus_ = msg->status;
    return;
}

void Upgrade::mcuVerCallBack(const std_msgs::String::ConstPtr& msg)
{
    HJ_CST_TIME_DEBUG(ota_logger, "receive mcu ver: %s\n", msg->data.c_str());
    mcuVerJson_ = nlohmann::json::parse(msg->data);
    return;
}

bool Upgrade::dowork(const boost::any& para)
{
    failModule_ = none;
    status_ = WORKING;
    failMsg_.empty();
    angoOta_ = 2;
    std::string info = boost::any_cast<std::string>(para);
    nlohmann::json baseJson(nlohmann::json::object());
    nlohmann::json ledJson(nlohmann::json::object());
    nlohmann::json angoJson(nlohmann::json::object());

    try {
        nlohmann::json cutRstJson = nlohmann::json::parse(info);
        baseJson = cutRstJson["mcu"]["baseboard"];
        ledJson = cutRstJson["mcu"]["ledboard"];
        angoJson = cutRstJson["mcu"]["ango"];
    } catch (const std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "json parse error\n");
        failMsg_ = "json parse error";
        return false;
    }
    
    if(!angoJson.contains("path") || !angoJson.contains("version")) {
        HJ_CST_TIME_ERROR(ota_logger, "ango json parse error\n");
        failMsg_ = "ango json parse error";
        return false;
    }

    if(!baseJson.contains("path") || !baseJson.contains("version")) {
        HJ_CST_TIME_ERROR(ota_logger, "base json parse error\n");
        failMsg_ = "base json parse error";
        return false;
    }

    if(!ledJson.contains("path") || !ledJson.contains("version")) {
        HJ_CST_TIME_ERROR(ota_logger, "led json parse error\n");
        failMsg_ = "led json parse error";
        return false;
    }

    int abcnt = 0;
    while (airBagStatus_ != 0) {
        HJ_CST_TIME_DEBUG(ota_logger, "air bag release not complete:%d\n", airBagStatus_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++abcnt;
        if (abcnt >= 50) {
            HJ_CST_TIME_ERROR(ota_logger, "air bag release time out\n");
            failMsg_ = "air bag release time out";
            return false;
        }
    }
    HJ_CST_TIME_DEBUG(ota_logger, "air bag check done:%d\n", airBagStatus_);

    if (mcuVerJson_.contains("ango_ver") && 
        (mcuVerJson_["ango_ver"].get<std::string>() == angoJson["version"].get<std::string>())) {   
        HJ_CST_TIME_DEBUG(ota_logger, "ango ver equals: [%s] [%s], skip upgrade\n",
            mcuVerJson_["ango_ver"].get<std::string>().c_str(),
            angoJson["version"].get<std::string>().c_str());
    } else if (!upgradeAngo(angoJson["path"], angoJson["version"])) {
        HJ_CST_TIME_ERROR(ota_logger, "ota ango fail!\n");
        angoOta_ = 0;
        //failModule_ = ango;
        //return false;
    } else {
        HJ_CST_TIME_DEBUG(ota_logger, "ota ango success!\n");
        angoOta_ = 1;
    }
    otaRptFunc_(2, 75, "", 0);
    

    if (mcuVerJson_.contains("led_ver") && 
        (mcuVerJson_["led_ver"].get<std::string>() == ledJson["version"].get<std::string>())) {   
        HJ_CST_TIME_DEBUG(ota_logger, "led ver equals: [%s] [%s], skip upgrade\n",
            mcuVerJson_["led_ver"].get<std::string>().c_str(),
            ledJson["version"].get<std::string>().c_str());
    } else if (!upgradeLed(ledJson["path"], ledJson["version"])) {
        HJ_CST_TIME_ERROR(ota_logger, "ota led fail!\n");
        failModule_ = ledboard;
        failMsg_ = curOtaRet_.msg_;
        return false;
    }
    otaRptFunc_(2, 80, "", 0);
    HJ_CST_TIME_DEBUG(ota_logger, "ota led success!\n");

    if (!upgradeBase(baseJson["path"], baseJson["version"])) {
        HJ_CST_TIME_ERROR(ota_logger, "ota base fail!\n");
        failModule_ = baseboard;
        failMsg_ = curOtaRet_.msg_;
        doUpgradeBack(ledboard);
        return false;
    }
    otaRptFunc_(2, 90, "", 0);
    HJ_CST_TIME_DEBUG(ota_logger, "ota base success!\n");
    
    if (!upgradeSoc()) {
        HJ_CST_TIME_ERROR(ota_logger, "ota soc fail!\n");
        failModule_ = soc;
        failMsg_ = curOtaRet_.msg_;
        doUpgradeBack(none);
        return false;
    }
    otaRptFunc_(2, 95, "", 0);
    HJ_CST_TIME_DEBUG(ota_logger, "ota soc success!\n");

    return true;
}

bool Upgrade::doUpgradeBack(OtaModule module)
{
    //none means all
    nlohmann::json etcVersion(nlohmann::json::object());
    std::string basever, ledver;
    std::string basepath = "/data/hj/basemcu.bin";
    std::string ledpath = "/data/hj/ledmcu.bin";
    
    HJ_CST_TIME_DEBUG(ota_logger, "trigger roll back: %d!\n", (int)module);

    try {
        std::ifstream versionFile("/etc/version");
        if (!versionFile.is_open()) {
            HJ_CST_TIME_ERROR(ota_logger, "open /etc/version fail\n")
            return false;
        }
        versionFile >> etcVersion;
        basever = etcVersion["basemcu_ver"];
        ledver = etcVersion["ledmcu_ver"];
    } catch (const std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "parse etc version fail\n");
        return false;
    }

    if (!utils::isFileExist(basepath.data())) {
        HJ_CST_TIME_ERROR(ota_logger, "%s not exist\n", basepath.c_str());
        return false;
    }

    if (!utils::isFileExist(ledpath.data())) {
        HJ_CST_TIME_ERROR(ota_logger, "%s not exist\n", ledpath.c_str());
        return false;
    }

    status_ = WORKING;

    if (module == none) {
        if (!upgradeLed(ledpath, ledver)) {
            HJ_CST_TIME_ERROR(ota_logger, "roll back led mcu fail\n");
            return false;
        }
        HJ_CST_TIME_DEBUG(ota_logger, "roll back led mcu to [%s] success\n", ledver.c_str());

        if (!upgradeBase(basepath, basever)) {
            HJ_CST_TIME_ERROR(ota_logger, "roll back base mcu fail\n");
            //upgradeLed(ledpath, ledver);
            return false;
        }
        HJ_CST_TIME_DEBUG(ota_logger, "roll back base mcu to [%s] success\n", basever.c_str());

        HJ_CST_TIME_DEBUG(ota_logger, "roll back all success\n");
    } else if (module == baseboard) {
        if (!upgradeBase(basepath, basever)) {
            HJ_CST_TIME_ERROR(ota_logger, "roll back base mcu fail\n");
            return false;
        }
        HJ_CST_TIME_DEBUG(ota_logger, "roll back base mcu to [%s] success\n", basever.c_str());
    } else if (module == ledboard) {
        if (!upgradeLed(ledpath, ledver)) {
            HJ_CST_TIME_ERROR(ota_logger, "roll back led mcu fail\n");
            return false;
        }
        HJ_CST_TIME_DEBUG(ota_logger, "roll back led mcu to [%s] success\n", ledver.c_str());
    }

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
    HJ_CST_TIME_DEBUG(ota_logger, "ango ota...\n");

    while (--retry >= 0) {
        otaPub_.publish(msg);

        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
            return curOtaRet_.module_ == ango;
        })) {
            HJ_CST_TIME_ERROR(ota_logger, "ango ota timeout!\n");
            continue;
        } 

        if (curOtaRet_.ret_ != 0) {
            HJ_CST_TIME_ERROR(ota_logger, "ango ota fail: %s!\n", curOtaRet_.msg_.c_str());
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
    HJ_CST_TIME_DEBUG(ota_logger, "led ota...\n");

    while (--retry >= 0) {
        otaPub_.publish(msg);

        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
            return curOtaRet_.module_ == ledboard;
        })) {
            HJ_CST_TIME_ERROR(ota_logger, "led ota timeout!\n");
            continue;
        } 

        if (curOtaRet_.ret_ != 0) {
            HJ_CST_TIME_ERROR(ota_logger, "led ota fail: %s!\n", curOtaRet_.msg_.c_str());
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
    HJ_CST_TIME_DEBUG(ota_logger, "base ota...\n");

    while (--retry >= 0) {
        otaPub_.publish(msg);

        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::minutes(3), [&]() {
            return curOtaRet_.module_ == baseboard;
        })) {
            HJ_CST_TIME_ERROR(ota_logger, "baseboard ota timeout!\n");
            continue;
        } 

        if (curOtaRet_.ret_ != 0) {
            HJ_CST_TIME_ERROR(ota_logger, "baseboard ota fail: %s!\n", curOtaRet_.msg_.c_str());
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
        HJ_CST_TIME_ERROR(ota_logger, "soc ota timeout!\n");
        return false;
    } 

    if (curOtaRet_.ret_ != 0) {
        HJ_CST_TIME_ERROR(ota_logger, "soc ota fail: %s!\n", curOtaRet_.msg_.c_str());
        return false;
    }

    return true;
}

void Upgrade::upgradeCb(const hj_interface::OtaUpgradeStatus::ConstPtr& msg)
{
    if (status_ != WORKING) {
        HJ_CST_TIME_DEBUG(ota_logger, "upgrade not working\n");
        return;
    }

    HJ_CST_TIME_DEBUG(ota_logger, "[ota resp] todo = %s\n", msg->todo.c_str());
    HJ_CST_TIME_DEBUG(ota_logger, "[ota resp] stage = %d\n", msg->stage);
    HJ_CST_TIME_DEBUG(ota_logger, "[ota resp] module = %d\n", msg->module);
    HJ_CST_TIME_DEBUG(ota_logger, "[ota resp] ret = %d\n", msg->ret);
    HJ_CST_TIME_DEBUG(ota_logger, "[ota resp] msg = %s\n", msg->msg.c_str());

    std::unique_lock<std::mutex> lc(mtx_);
    curOtaRet_.module_ = static_cast<OtaModule>(msg->module);
    curOtaRet_.ret_ = msg->ret;
    curOtaRet_.msg_ = msg->msg;
    cond_.notify_one();
}

}//namespace aiper_ota