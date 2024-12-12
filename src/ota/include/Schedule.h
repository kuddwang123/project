#pragma once
#include "State/DownLoad.h"
#include "State/UnPack.h"
#include "State/Upgrade.h"
#include "State/End.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "nlohmann/json.hpp"
#include <condition_variable>
#include "hj_interface/AppMsg.h"
#include "hj_interface/AppOnlineType.h"
#include <deque>
#include <atomic>
#include <thread>

namespace aiper_ota {
class Schedule {
public:
    Schedule(ros::NodeHandle n);
    ~Schedule();
    void construct();
    void run();
    static const char* cloudVerFile_;
    
    typedef struct {
        std::deque<hj_interface::AppMsg> msgQue_;
        std::mutex mtx_;
    }AppMsgCacher;

private:
    ros::NodeHandle n_;
    ros::Subscriber urlTriggerSub_;
    ros::Subscriber otaEnterSub_;
    ros::Subscriber mcuVerSub_;
    ros::Subscriber appOnlineSub_;
    ros::Publisher otaPeportPub_;
    ros::Publisher otaAppResPub_;
    ros::Publisher otaEnterPub_;
    ros::Publisher otaQuitPub_;
    
    DownloadPtr dowloadPtr_;
    UnPackPtr unpackPtr_;
    UpgradePtr upGradePtr_;
    EndPtr_ endPtr_;

    std::string sn_;
    std::string socVer_;
    int mode_;
    std::atomic<uint8_t> otaPermit_;
    std::atomic<bool> runflag_;
    std::atomic<bool> appOnline_;
    std::atomic<bool> autoDlRun_;
    hj_interface::AppMsg otaAppMsg_;
    nlohmann::json urlOtaJson_;
    nlohmann::json mcuVerJson_;
    CloudVersion cloudVer_; //UrlOta下达时更新并存文件

    std::thread workThread_;
    std::condition_variable cond_;
    std::mutex mtx_;
    AppMsgCacher otaStatusRptCacher_;
    AppMsgCacher otaResultRptCacher_;

private:
    void otaStatusRptCb(int state, int progress, std::string msg, int error);
    void otaStatusReportAfterReboot(int state, int progress, std::string msg, int error);
    void otaResultRptCb(int state, int angostate, int ledstate, int basestate, int socstate, std::string msg, int error);
    void otaResultRptAfterReboot(int state, int angostate, int ledstate, int basestate, int socstate, const std::string& msg, int error);
    void otaEnterCallBack(const std_msgs::UInt8::ConstPtr& msg);
    void urlOtaTriggerCallBack(const hj_interface::AppMsg::ConstPtr& msg);
    void mcuVerCallBack(const std_msgs::String::ConstPtr& msg);
    void AppOnlineCb(const hj_interface::AppOnlineType::ConstPtr& msg);
    void doOtaThread();
    void otaCheck();
    bool saveCloudVerFile();
    void doManualOta();
    void doAutoOta();
    void doRollBackOta();
    void reportAppMsgCache();
    void urlOtaResp(const hj_interface::AppMsg& appMsg, int res);
};
}//namespace aiper_ota