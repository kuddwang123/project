#include "PostTunnel.h"
#include "log.h"
#include "hj_interface/SysAction.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_posttunnel::PostTunnel>(FUNCTION_NAME);
}
namespace collect_node_posttunnel {
PostTunnel::PostTunnel(const rapidjson::Value& json_conf): 
    hj_bf::Function(json_conf) 
{
    uint32_t queueSize = 100;
    uint32_t loadIntervalSec = 60;

    if (json_conf.HasMember("queueSize") && json_conf["queueSize"].IsUint()) {
        queueSize = json_conf["queueSize"].GetUint();
    }

    if (json_conf.HasMember("loadFailSec") && json_conf["loadFailSec"].IsUint()) {
        loadIntervalSec = json_conf["loadFailSec"].GetUint();
    }

    resetSub_ = hj_bf::HJSubscribe("collect_node/notify/func", 1, &PostTunnel::collectNodeResetCb, this);

    resetResPub_ = hj_bf::HJAdvertise<hj_interface::CollectBroadcast>("func/response/collect_node", 1);

    reporterPtr_ = std::unique_ptr<Reporter>(new Reporter(queueSize, loadIntervalSec));
    assert(reporterPtr_);

    reporterPtr_->start();
} 

PostTunnel::~PostTunnel()
{
    reporterPtr_->stop();
}

void PostTunnel::collectNodeResetCb(const hj_interface::CollectBroadcast::ConstPtr& msg)
{
    HJ_INFO("posttunnel receive reset, act:%d\n", msg->action);
   
    if (msg->action == hj_interface::SysAction::SYS_ACTION_RESTORE_FACTORY) {
        HJ_INFO("posttunnel reset\n");
        reporterPtr_->reset();
    } else if (msg->action == hj_interface::SysAction::SYS_ACTION_SHUTDOWN ||
        msg->action == hj_interface::SysAction::SYS_ACTION_SLEEP) {
        HJ_INFO("posttunnel stop\n");
        reporterPtr_->stop();
    }

    hj_interface::CollectBroadcast response_msg;
    response_msg.action = msg->action;
    response_msg.ack = 0;
    response_msg.function = 2;
    resetResPub_.publish(response_msg);
}

} //namespace collect_node_posttunnel