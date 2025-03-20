#pragma once 

#include <ros/ros.h>
#include <boost/any.hpp>
#include "Common.h"

namespace aiper_ota {
enum work_state {
    READY,
    WORKING,
    END
};

class BaseState
{
public:
    BaseState(ros::NodeHandle n, const otaStatusReportFunc& otaRptFunc):
        n_(n), otaRptFunc_(otaRptFunc), status_(READY) {};
    virtual ~BaseState() {};
    bool virtual dowork(const boost::any& value) = 0;
    virtual work_state getStatus() {return status_;}

protected:    
    ros::NodeHandle n_;
    work_state status_;
    otaStatusReportFunc otaRptFunc_;
};

typedef std::shared_ptr<BaseState> BaseStatePtr;

}//aiper_otas