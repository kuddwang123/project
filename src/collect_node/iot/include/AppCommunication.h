/**
 * @ Author: lou.pan
 * @ Create Time: 2024-04-18 16:47:34
 * @ Modified by: lou.pan
 * @ Modified time: 2024-06-13 18:32:25
 * @ Description: hj function entrance class
 */

#ifndef INCLUDE_APP_COMMUNICATION_H
#define INCLUDE_APP_COMMUNICATION_H
#include "function_factory.h"
#include "node_factory.h"
#include "AppDataRouter.h"
#include "std_msgs/String.h"
#include "NetConfig.h"
#include <memory>
#include <atomic>
#include <thread>
#include "Utils.h"

namespace collect_node_iot{

class AppCommunication : public hj_bf::Function {
  public:
    explicit AppCommunication(const rapidjson::Value& json_conf);
    ~AppCommunication();
  
  private:
    std::shared_ptr<AppDataRouter> appDataRouterPtr_;
    std::unique_ptr<NetConfig> netCfgPtr_;
    hj_bf::HJSubscriber appDataSub_;
    hj_bf::HJPublisher s3urlPub_;
    hj_bf::HJPublisher timePub_;
    hj_bf::HJPublisher getLogPub_;
    std::string certdir_;
    std::string certfile_;
    std::string certfilename_;
    std::string tzmdir_;
    std::string tzmfile_;
    int port_;

  private:
    void initialize();
    bool appDataHandler(const hj_interface::AppMsg&);
    void middleWareDataCb(const std_msgs::String&);
    void initIot();
    bool isCertFileExist();
    std::string timeZoneSet(const std::string& timeStr, const std::string& timeZoneOffset);
};

}
#endif