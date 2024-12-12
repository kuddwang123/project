#pragma once

#include "BaseState.h"

namespace aiper_ota {

class End: public BaseState {

public:
    End(ros::NodeHandle n, const otaResultReportFunc& otaRstRptFunc, const otaStatusReportFunc& otaRptFunc);
    bool dowork(const boost::any& para) override;

private:
    bool handleDownLoadFail();
    bool handleUnPackFail();
    bool handleUpgradeSucc();
    bool handleUpgradeFail(OtaModule);
    void pubMcuReset();
    void pubOtaQuit();

private:
    ros::Publisher resetMcuPub_;
    ros::Publisher quitOtaPub_;
    otaResultReportFunc otaRstRptFunc_;
};

typedef std::shared_ptr<End> EndPtr_;

}//namespace aiper_ota