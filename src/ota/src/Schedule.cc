#include "ros/ros.h"
#include "Schedule.h"
#include "Common.h"
#include "hjlog.h"
#include <boost/any.hpp>
#include "boost/filesystem.hpp"

namespace aiper_ota {
const char* Schedule::cloudVerFile_ = "/userdata/logic/cloud_version.json";

Schedule::Schedule(ros::NodeHandle n):
    n_(n),
    runflag_(false),
    urlOtaJson_(nlohmann::json::object()),
    mcuVerJson_(nlohmann::json::object()),
    mode_(-1),
    otaPermit_(255),
    appOnline_(false),
    autoDlRun_(false)
{
    utils::DevInfo devinfo("/tmp/devInfo.json");
    sn_ = devinfo.sn();
    socVer_ = devinfo.version();

    urlTriggerSub_ = n_.subscribe("aiper_ota_trigger", 1, &Schedule::urlOtaTriggerCallBack, this);
    otaEnterSub_ = n_.subscribe("OtaEnterResp", 1, &Schedule::otaEnterCallBack, this);
    mcuVerSub_ = n_.subscribe("mcuVer_chatter", 1, &Schedule::mcuVerCallBack, this);
    appOnlineSub_ = n_.subscribe("/AppOnline", 1, &Schedule::AppOnlineCb, this);

    otaAppResPub_ = n_.advertise<hj_interface::AppMsg>("RespToApp" , 10);
    otaPeportPub_ = n_.advertise<hj_interface::AppMsg>("ReportApp", 10);
    otaEnterPub_ = n_.advertise<std_msgs::UInt8>("OtaEnterReq", 1);
    otaQuitPub_ = n_.advertise<std_msgs::UInt8>("OtaQuit", 1);

    while (otaAppResPub_.getNumSubscribers() == 0 || 
        otaPeportPub_.getNumSubscribers() == 0) {
        HJ_CST_TIME_DEBUG(ota_logger, "ros pub init...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    HJ_CST_TIME_DEBUG(ota_logger, "ros pub init done\n");

    boost::filesystem::path dir("/userdata/logic");
    if (!boost::filesystem::is_directory(dir)) {
        assert(boost::filesystem::create_directories(dir));
    }
}

Schedule::~Schedule()
{
    if (workThread_.joinable()) {
        workThread_.join();
    }
}

void Schedule::construct()
{
    auto statusRptCb = boost::bind(&Schedule::otaStatusRptCb, this,
        boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4);
    auto resultRptCb = boost::bind(&Schedule::otaResultRptCb, this,
        boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, 
        boost::placeholders::_5, boost::placeholders::_6, boost::placeholders::_7);

    dowloadPtr_ = std::make_shared<Download>(n_, statusRptCb);
    unpackPtr_ = std::make_shared<Unpack>(n_, statusRptCb, 3 * 60);
    upGradePtr_ = std::make_shared<Upgrade>(n_, statusRptCb);
    endPtr_ = std::make_shared<End>(n_, resultRptCb, statusRptCb);
}

void Schedule::AppOnlineCb(const hj_interface::AppOnlineType::ConstPtr& msg)
{
    if (!appOnline_.load() && (msg->type == hj_interface::AppOnlineType::IOT || 
            msg->type == hj_interface::AppOnlineType::BT_IOT)) {
        HJ_CST_TIME_DEBUG(ota_logger, "set ota app online true\n");
        appOnline_.store(true);
        reportAppMsgCache();
    } else if (msg->type == hj_interface::AppOnlineType::OFFLINE || 
            msg->type == hj_interface::AppOnlineType::BT) {
        HJ_CST_TIME_DEBUG(ota_logger, "set ota app online false\n");
        appOnline_.store(false);
    }
}

void Schedule::reportAppMsgCache()
{
    {
        std::unique_lock<std::mutex> lk(otaStatusRptCacher_.mtx_);
        if (!otaStatusRptCacher_.msgQue_.empty()) {
            auto appmsg = otaStatusRptCacher_.msgQue_.front();
            otaPeportPub_.publish(appmsg);
            otaStatusRptCacher_.msgQue_.pop_front();
        }
    }

    {
        std::unique_lock<std::mutex> lk(otaResultRptCacher_.mtx_);
        if (!otaResultRptCacher_.msgQue_.empty()) {
            auto appmsg = otaResultRptCacher_.msgQue_.front();
            otaPeportPub_.publish(appmsg);
            otaResultRptCacher_.msgQue_.pop_front();
        }
    }
}

void Schedule::run()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}

void Schedule::otaStatusRptCb(int state, int progress, std::string msg, int error)
{
    //HJ_CST_TIME_DEBUG(ota_logger, "in otaStatusRptCb\n");
    std::string version;
    uint64_t controlId = 0;
    try {
        version = urlOtaJson_["version"];
    } catch(std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "parse version fail, use current version\n");
        version = std::string("V") + socVer_;
        //return;
    }
/*
    if (urlOtaJson_.contains("controlLogId")) {
        controlId = urlOtaJson_["controlLogId"].get<uint64_t>();
    }
*/
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = "OtaStatusReport";
    
    nlohmann::json data(nlohmann::json::object());
    data["progress"] = progress;
    data["sn"] = sn_;
    data["version"] = version;
    data["state"] = state;
    data["error"] = error;
    data["msg"] = msg;
    appdata.payload = data.dump();

    appmsg.appdata.emplace_back(appdata);
    appmsg.to = hj_interface::AppMsg::SHADOW;

    if (appOnline_.load()) {
        otaPeportPub_.publish(appmsg);
    } else {
        std::unique_lock<std::mutex> lk(otaStatusRptCacher_.mtx_);
        otaStatusRptCacher_.msgQue_.clear();
        otaStatusRptCacher_.msgQue_.emplace_back(appmsg);
    }
}

void Schedule::otaStatusReportAfterReboot(int state, int progress, std::string msg, int error)
{
    std::string version = cloudVer_.nextVer_;
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = "OtaStatusReport";
    
    nlohmann::json data(nlohmann::json::object());
    data["progress"] = progress;
    data["sn"] = sn_;
    data["version"] = version;
    data["state"] = state;
    data["error"] = error;
    data["msg"] = msg;
    appdata.payload = data.dump();

    appmsg.appdata.emplace_back(appdata);
    appmsg.to = hj_interface::AppMsg::SHADOW;

    if (appOnline_.load()) {
        otaPeportPub_.publish(appmsg);
    } else {
        std::unique_lock<std::mutex> lk(otaStatusRptCacher_.mtx_);
        otaStatusRptCacher_.msgQue_.clear();
        otaStatusRptCacher_.msgQue_.emplace_back(appmsg);
    }
}

void Schedule::otaResultRptCb(int state, int angostate, int ledstate, int basestate, int socstate, std::string msg, int error)
{
    nlohmann::json rpt(nlohmann::json::object());
    nlohmann::json angoJs(nlohmann::json::object());
    nlohmann::json ledJs(nlohmann::json::object());
    nlohmann::json mcuJs(nlohmann::json::object());
    nlohmann::json socJs(nlohmann::json::object());

    socJs["state"] = socstate;
    socJs["version"] = socVer_;
    
    mcuJs["state"] = basestate;
    mcuJs["version"] = mcuVerJson_["base_ver"];

    ledJs["state"] = ledstate;
    ledJs["version"] = mcuVerJson_["led_ver"];

    angoJs["state"] = angostate;
    angoJs["version"] = mcuVerJson_["ango_ver"];
    
    rpt["sn"] = sn_;
    rpt["cloudver"] = urlOtaJson_.contains("version") ?
         urlOtaJson_["version"].get<std::string>() : "";
    rpt["state"] = state;
    rpt["msg"] = msg;
    rpt["controlLogId"] = urlOtaJson_.contains("controlLogId") ?
        urlOtaJson_["controlLogId"].get<uint64_t>() : 0;
    rpt["socver"] = socJs;
    rpt["mcu"] = mcuJs;
    rpt["led"] = ledJs;
    rpt["ango"] = angoJs;
    rpt["error"] = error;

    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = "OtaResultReport";
    appdata.payload = rpt.dump();

    appmsg.appdata.emplace_back(appdata);
    appmsg.to = hj_interface::AppMsg::CLOUD;

    if (appOnline_.load()) {
        otaPeportPub_.publish(appmsg);
    } else {
        std::unique_lock<std::mutex> lk(otaResultRptCacher_.mtx_);
        otaResultRptCacher_.msgQue_.clear();
        otaResultRptCacher_.msgQue_.emplace_back(appmsg);
    }
}

void Schedule::otaResultRptAfterReboot(int state, int angostate, int ledstate, int basestate, int socstate, const std::string& msg, int error)
{
    nlohmann::json rpt(nlohmann::json::object());
    nlohmann::json angoJs(nlohmann::json::object());
    nlohmann::json ledJs(nlohmann::json::object());
    nlohmann::json mcuJs(nlohmann::json::object());
    nlohmann::json socJs(nlohmann::json::object());

    socJs["state"] = socstate;
    socJs["version"] = socVer_;
    
    mcuJs["state"] = basestate;
    mcuJs["version"] = mcuVerJson_["base_ver"];

    ledJs["state"] = ledstate;
    ledJs["version"] = mcuVerJson_["led_ver"];

    angoJs["state"] = angostate;
    angoJs["version"] = mcuVerJson_["ango_ver"];
    
    rpt["sn"] = sn_;
    rpt["cloudver"] = cloudVer_.nextVer_;
    rpt["state"] = state;
    rpt["msg"] = msg;
    rpt["controlLogId"] = cloudVer_.controlLogId_;
    rpt["socver"] = socJs;
    rpt["mcu"] = mcuJs;
    rpt["led"] = ledJs;
    rpt["ango"] = angoJs;
    rpt["error"] = error;

    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = "OtaResultReport";
    appdata.payload = rpt.dump();

    appmsg.appdata.emplace_back(appdata);
    appmsg.to = hj_interface::AppMsg::CLOUD;

    if (appOnline_.load()) {
        otaPeportPub_.publish(appmsg);
    } else {
        std::unique_lock<std::mutex> lk(otaResultRptCacher_.mtx_);
        otaResultRptCacher_.msgQue_.clear();
        otaResultRptCacher_.msgQue_.emplace_back(appmsg);
    }
}

void Schedule::urlOtaResp(const hj_interface::AppMsg& appMsg, int res)
{
    nlohmann::json urlota = nlohmann::json::parse(appMsg.appdata.at(0).payload);
    nlohmann::json subobj(nlohmann::json::object());
    subobj["controlLogId"] = urlota.contains("controlLogId") ? urlota["controlLogId"].get<uint64_t>() : 0;

    hj_interface::AppMsg resp;
    hj_interface::AppData appdata;
    resp.from = appMsg.from;
    appdata.res = res;
    appdata.key = appMsg.appdata.at(0).key;
    appdata.payload = subobj.dump();
    resp.appdata.emplace_back(appdata);

    otaAppResPub_.publish(resp);

    return;
}

void Schedule::urlOtaTriggerCallBack(const hj_interface::AppMsg::ConstPtr& msg)
{
    HJ_CST_TIME_DEBUG(ota_logger, "receive url ota:\n%s\n", msg->appdata.at(0).payload.c_str());

    if (runflag_.load()) {
        HJ_CST_TIME_DEBUG(ota_logger, "Ota is running: [%d]\n", mode_);        
        if (mode_ == 0) {
            urlOtaResp(*msg.get(), -1);
            HJ_CST_TIME_DEBUG(ota_logger, "Manual ota is running, return\n");
            return;
        } else if (mode_ == 1 && otaPermit_ == 1) {
            urlOtaResp(*msg.get(), -1);
            HJ_CST_TIME_DEBUG(ota_logger, "Auto ota is deep running, return\n");
            return;
        } else if (mode_ == 1 && dowloadPtr_->getStatus() == WORKING) {
            //cancel download
            autoDlRun_.store(false);
            dowloadPtr_->stop();
            if (workThread_.joinable()) {
                workThread_.join();
            }
            HJ_CST_TIME_DEBUG(ota_logger, "Last auto ota stoped!\n");
        } else {
            urlOtaResp(*msg.get(), -1);
            HJ_CST_TIME_DEBUG(ota_logger, "Ota is running, return\n");
            return;
        }
    }

    runflag_.store(true);
    otaAppMsg_ = *msg.get();

    try {
        urlOtaJson_ = nlohmann::json::parse(msg->appdata.at(0).payload);
        mode_ = urlOtaJson_["sites"]["type"];
        std::string md5 = urlOtaJson_["sites"]["md5"].get<std::string>();
        std::string url = urlOtaJson_["sites"]["url"].get<std::string>();
        std::string nextver = urlOtaJson_["version"];

        if (md5.empty() || url.empty() || nextver.empty()) {
            HJ_CST_TIME_DEBUG(ota_logger, "download para invalid:[%s] [%s] [%s]\n", 
                md5.c_str(), url.c_str(), nextver.c_str());
            runflag_.store(false);
            urlOtaResp(*msg.get(), -1);
            return;
        }

        if (workThread_.joinable()) {
            workThread_.join();
        }
        workThread_ = std::thread(&Schedule::doOtaThread, this);
    } catch (const std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "Json parse fail\n");
        runflag_.store(false);
        urlOtaResp(*msg.get(), -1);
        return;
    }

    return;
}

void Schedule::doOtaThread()
{
    std_msgs::UInt8 msg;
    EndPara endpara;
    DlPara dlpara;
    dlpara.mode_ = mode_;
    dlpara.md5_ = urlOtaJson_["sites"]["md5"].get<std::string>();
    dlpara.url_ = urlOtaJson_["sites"]["url"].get<std::string>();
    dlpara.nextver_ = urlOtaJson_["version"];
    
    uintmax_t userdataAvail = utils::getAvailDiskSpace("/userdata");
    if (userdataAvail <= urlOtaJson_["sites"]["fileSize"].get<uint64_t>()) {
       HJ_CST_TIME_ERROR(ota_logger, "no enough in userdata: %lld\n", userdataAvail);
       urlOtaResp(otaAppMsg_, -1);
       runflag_.store(false);
       return; 
    }

    HJ_CST_TIME_DEBUG(ota_logger, "userdata available size check pass!\n");

    if (mode_ == 0) { //手动
        doManualOta();
    } else if (mode_ == 1) { //自动
        urlOtaResp(otaAppMsg_, 0);
        doAutoOta();
    } else {
        urlOtaResp(otaAppMsg_, -1);
        runflag_.store(false);
        HJ_CST_TIME_ERROR(ota_logger, "unknown work mode: %d\n", mode_);
    }
    return;
}

void Schedule::doManualOta()
{
    std_msgs::UInt8 msg;
    EndPara endpara;
    DlPara dlpara;
    dlpara.mode_ = mode_;
    dlpara.md5_ = urlOtaJson_["sites"]["md5"].get<std::string>();
    dlpara.url_ = urlOtaJson_["sites"]["url"].get<std::string>();
    dlpara.nextver_ = urlOtaJson_["version"].get<std::string>();

    HJ_CST_TIME_DEBUG(ota_logger, "ota permit pub\n");
    otaEnterPub_.publish(msg);
    {
        std::unique_lock<std::mutex> lc(mtx_);
        if (!cond_.wait_for(lc, std::chrono::seconds(5), [&](){
            return otaPermit_ != 255;     
        })) {
            HJ_CST_TIME_ERROR(ota_logger, "ota permit timeout\n");
            otaPermit_ = 255;
            runflag_.store(false);
            urlOtaResp(otaAppMsg_, -1);
            return;
        }
    }
    
    if (otaPermit_ != 1) {
        HJ_CST_TIME_ERROR(ota_logger, "ota permit false: %d\n", otaPermit_.load());
        otaPermit_ = 255;
        runflag_.store(false);
        urlOtaResp(otaAppMsg_, -1);
        return;
    }

    urlOtaResp(otaAppMsg_, 0);
    
    assert(saveCloudVerFile());

    dlpara.timeout_ = 35 * 60; //35min
    if (!dowloadPtr_->dowork(boost::any(dlpara))) {
        HJ_CST_TIME_ERROR(ota_logger, "download work fail\n");
        endpara.endState_ = DOWNLOAD_STATE;
        endPtr_->dowork(boost::any(endpara));
        otaPermit_ = 255;
        runflag_.store(false);
        return;
    }
    HJ_CST_TIME_DEBUG(ota_logger, "download work success\n");

    std::string nextver = urlOtaJson_["version"];
    if (!unpackPtr_->dowork(boost::any(nextver))) {
        HJ_CST_TIME_ERROR(ota_logger, "unpack work fail\n");
        endpara.endState_ = UNPACK_STATE;
        endPtr_->dowork(boost::any(endpara));
        otaPermit_ = 255;
        runflag_.store(false);
        return;
    }
    HJ_CST_TIME_DEBUG(ota_logger, "unpack work success\n");

    auto cutinfo = unpackPtr_->getCutMsg(); 
    if (!upGradePtr_->dowork(boost::any(cutinfo))) {
        HJ_CST_TIME_ERROR(ota_logger, "upgrade work fail\n");
        endpara.endState_ = UPGRADE_FAIL_STATE;
        endpara.failModule_ = upGradePtr_->getFailModule();
        endPtr_->dowork(boost::any(endpara));
        return;
    }
    HJ_CST_TIME_DEBUG(ota_logger, "upgrade work success\n");

    endpara.endState_ = UPGRADE_SUCC_STATE;
    endPtr_->dowork(boost::any(endpara));
    return;
}
    
void Schedule::doAutoOta()
{
    std_msgs::UInt8 msg;
    msg.data = 1;
    EndPara endpara;
    DlPara dlpara;
    dlpara.mode_ = mode_;
    dlpara.md5_ = urlOtaJson_["sites"]["md5"].get<std::string>();
    dlpara.url_ = urlOtaJson_["sites"]["url"].get<std::string>();
    dlpara.nextver_ = urlOtaJson_["version"];
    dlpara.timeout_ = -1;
    otaPermit_ = 255;

    assert(saveCloudVerFile());

    bool dlresult = false;
    autoDlRun_.store(true);
    while (!dlresult && autoDlRun_.load()) {
        dlresult = dowloadPtr_->dowork(boost::any(dlpara));
        if (!autoDlRun_.load()) {
            HJ_CST_TIME_DEBUG(ota_logger, "download work cancel\n");
            return;
        }
        if (!dlresult) {
            HJ_CST_TIME_ERROR(ota_logger, "auto ota download work fail, continue\n");
        }
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    HJ_CST_TIME_DEBUG(ota_logger, "download work success\n");

    otaStatusRptCb(2, 0, "", 0);

    while (runflag_.load()) {
        otaEnterPub_.publish(msg);
        HJ_CST_TIME_DEBUG(ota_logger, "enter ota pub\n");
        {
            std::unique_lock<std::mutex> lc(mtx_);
            if (!cond_.wait_for(lc, std::chrono::seconds(40), [&](){
                return otaPermit_ != 255;     
            })) {
                HJ_CST_TIME_ERROR(ota_logger, "ota permit timeout, retry\n");
                continue;
            }

            if (otaPermit_ == 1) {
                HJ_CST_TIME_DEBUG(ota_logger, "permit enter ota\n");
                otaStatusRptCb(2, 30, "", 0);
                sleep(5);
                break;
            } else {
                HJ_CST_TIME_DEBUG(ota_logger, "permit not enter ota\n");
                otaPermit_ = 255;
                std::this_thread::sleep_for(std::chrono::seconds(30));
            }
        }
    }

    std::string nextver = urlOtaJson_["version"];
    if (!unpackPtr_->dowork(boost::any(nextver))) {
        HJ_CST_TIME_ERROR(ota_logger, "unpack work fail\n");
        endpara.endState_ = UNPACK_STATE;
        endPtr_->dowork(boost::any(endpara));
        otaPermit_ = 255;
        runflag_.store(false);
        return;
    }
    HJ_CST_TIME_DEBUG(ota_logger, "unpack work success\n");

    auto cutinfo = unpackPtr_->getCutMsg(); 
    if (!upGradePtr_->dowork(boost::any(cutinfo))) {
        HJ_CST_TIME_ERROR(ota_logger, "upgrade work fail\n");
        endpara.endState_ = UPGRADE_FAIL_STATE;
        endpara.failModule_ = upGradePtr_->getFailModule();
        endPtr_->dowork(boost::any(endpara));
        return;
    }
    HJ_CST_TIME_DEBUG(ota_logger, "upgrade work success\n");

    endpara.endState_ = UPGRADE_SUCC_STATE;
    endPtr_->dowork(boost::any(endpara));
    return;
}

void Schedule::doRollBackOta()
{
    if (runflag_.load()) {
         HJ_CST_TIME_DEBUG(ota_logger, "ota already run\n");
         return;
    }

    runflag_.store(true);
    
    std::thread([&](){
        std_msgs::UInt8 msg;
        msg.data = 1;
        EndPara endpara;
        while (true) {
            otaEnterPub_.publish(msg);
            HJ_CST_TIME_DEBUG(ota_logger, "enter roll back ota pub\n");
            {
                std::unique_lock<std::mutex> lc(mtx_);
                if (!cond_.wait_for(lc, std::chrono::seconds(5), [&](){
                    return otaPermit_ != 255;     
                })) {
                    HJ_CST_TIME_ERROR(ota_logger, "ota permit timeout, retry\n");
                    continue;
                }

                if (otaPermit_ == 1) {
                    HJ_CST_TIME_DEBUG(ota_logger, "permit enter ota\n");
                    break;
                } else {
                    HJ_CST_TIME_DEBUG(ota_logger, "permit not enter ota\n");
                    otaPermit_ = 255;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }
        upGradePtr_->doUpgradeBack(none);
        HJ_CST_TIME_DEBUG(ota_logger, "roll back ota done\n");
        endpara.endState_ = UPGRADE_SUCC_STATE;
        endPtr_->dowork(endpara);
    }).detach();
}

bool Schedule::saveCloudVerFile()
{
    std::ofstream file(cloudVerFile_, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        HJ_CST_TIME_ERROR(ota_logger, "open cloud file fail\n");
        return false;
    }    

    nlohmann::json cloudVerJson(nlohmann::json::object());
    cloudVerJson["cur"] = std::string("V") + socVer_;
    cloudVerJson["next"] = urlOtaJson_.contains("version") ? urlOtaJson_["version"].get<std::string>() : "";
    cloudVerJson["controlLogId"] = urlOtaJson_.contains("controlLogId") ? urlOtaJson_["controlLogId"].get<uint64_t>() : 0;
    
    file << cloudVerJson.dump();
    file.close();

    HJ_CST_TIME_ERROR(ota_logger, "update cloud version success:%s\n", cloudVerJson.dump().c_str());
    return true;
}

void Schedule::otaCheck()
{
    if (!utils::isFileExist(Unpack::cutResultFile_)) {
        HJ_CST_TIME_DEBUG(ota_logger, "cut file not exist, last not ota\n");
        utils::emptyDir("/userdata/ota/");
        return;
    }

    assert(utils::isFileExist(cloudVerFile_));

    std::ifstream fwCutFile(Unpack::cutResultFile_);
    std::ifstream cloudVerFile(cloudVerFile_);
    assert(fwCutFile.is_open() && cloudVerFile.is_open());
    nlohmann::json cutResultJson;
    nlohmann::json cloudVerJson;

    try {
        fwCutFile >> cutResultJson;
        cloudVerFile >> cloudVerJson;
        cloudVer_.controlLogId_ = cloudVerJson["controlLogId"].get<uint64_t>();
        cloudVer_.curVer_ = cloudVerJson["cur"];
        cloudVer_.nextVer_ = cloudVerJson["next"];
    } catch (const std::exception& e) {
        HJ_CST_TIME_ERROR(ota_logger, "parse cut file error :%s\n", e.what());
        utils::emptyDir("/userdata/ota/");
        otaStatusRptCb(4, 100, "parse cut result fail",  2);
        otaResultRptCb(0, 0, 0, 0, 0, "", 2);
        saveCloudVerFile();
        return;
    }

    saveCloudVerFile();

    std::string curVer = std::string("V") + socVer_;
    std::string plandVer = cloudVerJson["next"];
    HJ_CST_TIME_DEBUG(ota_logger, "current version:%s, planed version:%s\n", 
        curVer.c_str(), plandVer.c_str());
    
    int totalstate = 0, socstate = 0, basestate = 0, ledstate = 0, angostate = 0;

    if (curVer != plandVer) {
        HJ_CST_TIME_DEBUG(ota_logger, "current version not equal with planed\n");
    } else {
        if (cutResultJson["fw_ver"].get<std::string>() == socVer_) {
            HJ_CST_TIME_DEBUG(ota_logger, "soc version check pass\n");
            socstate = 1;
        } else {
            HJ_CST_TIME_ERROR(ota_logger, "soc version check error: %s, %s\n",
            socVer_.c_str(), cutResultJson["fw_ver"].get<std::string>().c_str());
        }

        if (cutResultJson["mcu"]["baseboard"]["version"].get<std::string>() == 
                mcuVerJson_["base_ver"].get<std::string>()) {
            HJ_CST_TIME_DEBUG(ota_logger, "mcu base check pass\n");
            basestate = 1;
        } else {
            HJ_CST_TIME_ERROR(ota_logger, "basebord version check error: %s, %s\n",
            cutResultJson["mcu"]["baseboard"]["version"].get<std::string>().c_str(), 
            mcuVerJson_["base_ver"].get<std::string>().c_str());
        }

        if (cutResultJson["mcu"]["ledboard"]["version"].get<std::string>() == 
                mcuVerJson_["led_ver"].get<std::string>()) {
            HJ_CST_TIME_DEBUG(ota_logger, "mcu led check pass\n");
            ledstate = 1;
        } else {
            HJ_CST_TIME_ERROR(ota_logger, "led version check error: %s, %s\n",
            cutResultJson["mcu"]["ledboard"]["version"].get<std::string>().c_str(), 
            mcuVerJson_["led_ver"].get<std::string>().c_str());
        }
    /*
        if (cutResultJson["mcu"]["ango"]["version"].get<std::string>() == 
            mcuVerJson_["ango_ver"].get<std::string>()) {
            HJ_CST_TIME_DEBUG(ota_logger, "mcu ango check pass\n");
            angostate = 1;
        } else {
            HJ_CST_TIME_ERROR(ota_logger, "ango version check error: %s, %s\n",
            cutResultJson["mcu"]["ango"]["version"].get<std::string>().c_str(), 
            mcuVerJson_["ango_ver"].get<std::string>().c_str());
        }
    */  
        angostate = 1;
        totalstate = (socstate == 1 && basestate == 1 && ledstate == 1 && angostate == 1) ? 1 : 0;
    }

    HJ_CST_TIME_DEBUG(ota_logger, "total state: %d\n", totalstate);

    otaResultRptAfterReboot(totalstate, angostate, ledstate, basestate, socstate, "", (totalstate == 1 ? 0 : 2));
    otaStatusReportAfterReboot((totalstate == 1 ? 3 : 4), 100, totalstate == 1 ? "ota success" : "ota fail", (totalstate == 1 ? 0 : 2));
    
    utils::emptyDir("/userdata/ota/");

    if (socstate == 0) {
        HJ_CST_TIME_DEBUG(ota_logger, "soc ota fail, do mcu roll back\n");
        doRollBackOta();
    }

    return;
}

void Schedule::otaEnterCallBack(const std_msgs::UInt8::ConstPtr& msg)
{
    HJ_CST_TIME_DEBUG(ota_logger, "receive enter ota permit:%d\n", msg->data);
    if (!runflag_.load()) {
        HJ_CST_TIME_DEBUG(ota_logger, "not in ota running\n");
        return;
    }

    otaPermit_ = msg->data;
    cond_.notify_one();

    return;
}

void Schedule::mcuVerCallBack(const std_msgs::String::ConstPtr& msg)
{
    HJ_CST_TIME_DEBUG(ota_logger, "receive mcu ver: %s\n", msg->data.c_str());
    mcuVerJson_ = nlohmann::json::parse(msg->data);
    otaCheck();
    return;
}

}//namespace aiper_ota
