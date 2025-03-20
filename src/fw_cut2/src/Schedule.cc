#include "ros/ros.h"
#include "Schedule.h"
#include "Common.h"
#include "hj_interface/OtaUpgradeData.h"
#include <boost/any.hpp>
#include "boost/filesystem.hpp"
#include <signal.h>

namespace aiper_ota {

static bool runflag_g = true;
static bool ota_flag_g = false;
static void signalHandler(int signum) {
    //std::cout << "Interrupt signal (" << signum << ") received.\n";
    if (ota_flag_g) {
        return;
    }
    runflag_g = false;
    ros::shutdown();
}


Schedule::Schedule(ros::NodeHandle n):
    n_(n),
    runflag_(false),
    otaPermit_(0)
{
    otaEnterSub_ = n_.subscribe("OtaEnterResp", 1, &Schedule::otaEnterCallBack, this);
    otaEnterPub_ = n_.advertise<std_msgs::UInt8>("OtaEnterReq", 1);
    otaQuitPub_ = n_.advertise<std_msgs::UInt8>("OtaQuit", 1);
    resetMcuPub_ = n_.advertise<hj_interface::OtaUpgradeData>("/system/eventNotify", 1);

    while (otaEnterPub_.getNumSubscribers() == 0 || 
        otaQuitPub_.getNumSubscribers() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    signal(SIGINT, signalHandler);

    fprintf(stdout, "fw_cut init done, please make sure robot on charger and power > 30 or just touch /tmp/run_ota.mark\n");
}

Schedule::~Schedule()
{
    if (workThread_.joinable()) {
        workThread_.join();
    }
    ros::shutdown();
}

void Schedule::construct()
{
    auto statusRptCb = boost::bind(&Schedule::otaStatusRptCb, this,
        boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4);
    auto resultRptCb = boost::bind(&Schedule::otaResultRptCb, this,
        boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
        boost::placeholders::_4, boost::placeholders::_5);

    unpackPtr_ = std::make_shared<Unpack>(n_, statusRptCb, 3 * 60);
    upGradePtr_ = std::make_shared<Upgrade>(n_, statusRptCb);
}

void Schedule::run()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();

    workThread_ = std::thread(&Schedule::doOtaThread, this);

    ros::waitForShutdown();

    if (workThread_.joinable()) {
        workThread_.join();
    }
    utils::emptyDir("/userdata/ota/");
}

void Schedule::otaStatusRptCb(int state, int progress, std::string msg, int error)
{
    return;
}

void Schedule::otaResultRptCb(int state, int ledstate, int basestate, int socstate, std::string msg)
{
    return;
}

bool Schedule::doOta()
{
    ota_flag_g = true;
    if (!unpackPtr_->dowork(boost::any())) {
        fprintf(stderr, "unpack fail!\n");
        std_msgs::UInt8 msg;
        msg.data = 1;
        otaQuitPub_.publish(msg);
        ota_flag_g = false;
        return false;
    }
    fprintf(stdout, "unpack success!\n");

    auto cutinfo = unpackPtr_->getCutMsg(); 
    if (!upGradePtr_->dowork(boost::any(cutinfo))) {
        fprintf(stderr, "upgrade fail!\n");
        utils::emptyDir("/userdata/ota/");
        mcuReset();
        return false;
    }

    fprintf(stdout, "upgrade success!\n");
    utils::emptyDir("/userdata/ota/");
    mcuReset();
    return true;
}

void Schedule::mcuReset()
{
    hj_interface::OtaUpgradeData msg;
    msg.todo = "Reboot";
    msg.stage = 1;
    msg.module = baseboard;
    fprintf(stdout, "reset!\n");
    ::sync();
    resetMcuPub_.publish(msg);
}

void Schedule::doOtaThread()
{
    while(runflag_g && otaPermit_ == 0) {
        std_msgs::UInt8 msg;
        otaEnterPub_.publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    if (!runflag_g) {
        return;
    }

    fprintf(stdout, "enter ota!\n");
    doOta();
    sleep(2);
    ros::shutdown();
    return;
}

void Schedule::otaEnterCallBack(const std_msgs::UInt8::ConstPtr& msg)
{
    otaPermit_ = msg->data;
    return;
}

}//namespace aiper_ota
