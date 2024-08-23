#include "PostTunnel.h"

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
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

    reporterPtr_ = std::unique_ptr<Reporter>(new Reporter(queueSize, loadIntervalSec));
    assert(reporterPtr_);

    reporterPtr_->start();
} 

PostTunnel::~PostTunnel()
{
    reporterPtr_->stop();
}

} //namespace collect_node_posttunnel