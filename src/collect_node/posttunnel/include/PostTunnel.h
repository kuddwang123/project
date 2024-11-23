#pragma once

#include "function_factory.h"
#include "node_factory.h"
#include "Reporter.h"
#include "hj_interface/CollectBroadcast.h"
namespace collect_node_posttunnel {
class PostTunnel: public hj_bf::Function{
 public:
    explicit PostTunnel(const rapidjson::Value& json_conf);
    ~PostTunnel();
  
 private:
    std::unique_ptr<Reporter> reporterPtr_;
    hj_bf::HJSubscriber resetSub_;
    hj_bf::HJPublisher resetResPub_;
 
 private:
    void collectNodeResetCb(const hj_interface::CollectBroadcast::ConstPtr&);
};
} //namespace collect_node_posttunnel