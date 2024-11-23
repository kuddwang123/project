/**
 * @ Author: lou.pan
 * @ Create Time: 2024-04-16 20:11:44
 * @ Modified by: lou.pan
 * @ Modified time: 2024-05-23 15:06:28
 * @ Description: Handle mqtt event lifetime and wrap mqtt client
 */


#include "IotMqttClient.h"
#include <aws/crt/Types.h>
#include <string>
#include "node_factory.h"
#include "hj_interface/AppMsg.h"

namespace collect_node_iot {

struct IotMqttMsg {
    Aws::Crt::String uuid_;
    long double ts_;
    
    IotMqttMsg(Aws::Crt::String uuid, long double ts): 
        uuid_(uuid), ts_(ts) {}
};

class IotMqttClientLink {
typedef boost::function<void(const hj_interface::AppMsg&)> mqttReqFunc;
public:
    IotMqttClientLink();
    ~IotMqttClientLink();

  //初始化，向aws mqtt client注册事件回调
    bool initialize(const std::shared_ptr<IotMqttClient>&);
  
  //注册mqtt消息回调
    void setMqttReqCb(const mqttReqFunc& cb) {mqttReqFunc_ = cb;}

  //自定义topic响应app数据
    bool iotMqttResp(const hj_interface::AppMsg::ConstPtr&);

  //自定义topic上报
    bool iotMqttReport(const hj_interface::AppMsg& msg, uint8_t type);

  //shadow上报
    //void iotMqttUpdateShadow(const std::string&);

private:
    std::shared_ptr<IotMqttClient> iotMqttClientPtr_;
    mqttReqFunc mqttReqFunc_;

private:
    void mqttMsgIncome(const Aws::Crt::ByteBuf &, uint8_t);
};

}