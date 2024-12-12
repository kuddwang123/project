#include "State/End.h"
#include "hjlog.h"
#include "hj_interface/OtaUpgradeData.h"
#include "Utils.h"
#include "std_msgs/UInt8.h"
#include "Schedule.h"
#include <unistd.h>

namespace aiper_ota {
End::End(ros::NodeHandle n, const otaResultReportFunc& otaRstRptFunc, const otaStatusReportFunc& otaRptFunc):
    BaseState(n, otaRptFunc),
    otaRstRptFunc_(otaRstRptFunc)
{
    resetMcuPub_ = n_.advertise<hj_interface::OtaUpgradeData>("/system/eventNotify", 1);
    quitOtaPub_ = n_.advertise<std_msgs::UInt8>("OtaQuit", 1);
}

bool End::dowork(const boost::any& para)
{
    EndPara end = boost::any_cast<EndPara>(para);

    switch (end.endState_) {
        case DOWNLOAD_STATE:
            return handleDownLoadFail();
        
        case UNPACK_STATE:
            return handleUnPackFail();
        
        case UPGRADE_FAIL_STATE:
            return handleUpgradeFail(end.failModule_);
        
        case UPGRADE_SUCC_STATE:
            return handleUpgradeSucc();
        
        default:
            return false;
    }

    return false;
}

bool End::handleDownLoadFail()
{
    otaRptFunc_(4, 50, "Download bin fail", 1);
    otaRstRptFunc_(0, 0 ,0, 0, 0, "Download bin fail", 1);
    utils::removeFile(Schedule::cloudVerFile_);
    pubOtaQuit();

    return true;
}

bool End::handleUnPackFail()
{
    otaRptFunc_(4, 50 , "Unpack bin fail", 2);
    otaRstRptFunc_(0, 0, 0, 0, 0, "Unpack bin fail", 1);
    utils::removeFile(Schedule::cloudVerFile_);
    utils::emptyDir("/userdata/ota/");
    pubOtaQuit();

    return true;
}

bool End::handleUpgradeSucc()
{
    sleep(2);

    pubMcuReset();

    return true;
}

bool End::handleUpgradeFail(OtaModule failModule)
{
    //otaRptFunc_(4, 0 , "Upgrade fail", 2);

    switch(failModule) {
        case none:
            otaRstRptFunc_(0, 0, 0, 0, 0, "Air bag release timeout", 2);
            otaRptFunc_(4, 70 , "Air bag release timeout", 2);
            break;

        case ango:
            otaRstRptFunc_(0, 0, 0, 0, 0, "Upgrade ango fail", 2);
            otaRptFunc_(4, 70 , "Upgrade ango fail", 2);
            break;

        case ledboard:
            otaRstRptFunc_(0, 1, 0, 0, 0, "Upgrade led fail", 2);
            otaRptFunc_(4, 75 , "Upgrade led fail", 2);
            break;
        
        case baseboard:
            otaRstRptFunc_(0, 1, 1, 0, 0, "Upgrade baseboard fail", 2);
            otaRptFunc_(4, 80 , "Upgrade baseboard fail", 2);
            break;
        
        case soc:
            otaRstRptFunc_(0, 1, 1, 1, 0, "Upgrade soc fail", 2);
            otaRptFunc_(4, 90 , "Upgrade soc fail", 2);
            break;

        default:
            HJ_CST_TIME_ERROR(ota_logger, "unknown fail type: %d\n", failModule);
            otaRstRptFunc_(0, 0, 0, 0, 0, "Upgrade fail", 2);
    }

    utils::removeFile(Schedule::cloudVerFile_);

    utils::emptyDir("/userdata/ota/");

    pubOtaQuit();

    pubMcuReset();

    return true;
}

void End::pubMcuReset()
{
    hj_interface::OtaUpgradeData msg;
    msg.todo = "Reboot";
    msg.stage = 1;
    msg.module = baseboard;
    HJ_CST_TIME_DEBUG(ota_logger, "notify to reset mcu ota!\n");
    ::sync();
    resetMcuPub_.publish(msg);
}

void End::pubOtaQuit()
{
    std_msgs::UInt8 msg;
    msg.data = 1;
    HJ_CST_TIME_DEBUG(ota_logger, "notify to quit ota!\n");
    quitOtaPub_.publish(msg);
}

} //namespace aiper_ota