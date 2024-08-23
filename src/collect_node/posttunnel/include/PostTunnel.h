#pragma once

#include "function_factory.h"
#include "node_factory.h"
#include "Reporter.h"
namespace collect_node_posttunnel {
class PostTunnel: public hj_bf::Function{
 public:
    explicit PostTunnel(const rapidjson::Value& json_conf);
    ~PostTunnel();
  
 private:
    std::unique_ptr<Reporter> reporterPtr_;
};
} //namespace collect_node_posttunnel