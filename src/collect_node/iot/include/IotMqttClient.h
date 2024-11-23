/**
 * @ Author: lou.pan
 * @ Create Time: 2024-04-16 10:18:09
 * @ Modified by: lou.pan
 * @ Modified time: 2024-05-23 15:06:32
 * @ Description: Aws mqtt client SDK package
 */


#include <aws/crt/Api.h>
#include <aws/crt/StlAllocator.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/io/TlsOptions.h>

#include <aws/iot/MqttClient.h>

#include "AwsConnectionManager.h"

#include <boost/function.hpp>
#include <string>
#include <thread>
#include <atomic>

namespace collect_node_iot{
typedef boost::function<void(const Aws::Crt::ByteBuf &, uint8_t)> appMqttReqCb;

class IotMqttClient {
public:
    IotMqttClient(const std::string&);
    ~IotMqttClient();
  
  //Aws core连接成功槽函数
    void connectSlot();
  
  //Aws core断连成功槽函数
    void disconnSlot();
  
  //自定义topic上报app设备数据
    void mqttAppReport(const Aws::Crt::String&);

  //自定义topic响应app数据
    void mqttResp(const Aws::Crt::String&, uint8_t);
  
  //注册自定义topic请求回调
    void setAppMqttReqCb(const appMqttReqCb& cb) { appMqttCb_ = cb; }
  
  //上报shadow doc
    void updateShadowReport(const Aws::Crt::String&);
  
  //上报云端cloud设备数据
    void mqttCloudReport(const Aws::Crt::String&);
  
  //上报大数据
    void mqttBigDataReport(const Aws::Crt::String&);

private:
    enum SubIotSucc{
        SUB_IOT_TOPIC_ACK = 1 << 0,
        SUB_IOT_CLOUD_TOPIC_ACK = 1 << 1
    };

    std::string downChanTopic_;
    std::string upChanTopic_;
    std::string downChanCloudTopic_;
    std::string upChanCloudTopic_;
    std::string reportAppTopic_;
    std::string reportCloudTopic_;
    std::string reportShadowTopic_;
    std::string bigDataTopic_;
    std::atomic<bool> subRun_;
    std::atomic<uint8_t> subIotFlag_;
    std::thread subIotCoreThread_;

    appMqttReqCb appMqttCb_;

private:
    void subToDownChanThread();

    void subToDownChan();

    void subToCloudDownChan();

    void onDownChanMessageCb(Aws::Crt::Mqtt::MqttConnection &connection, 
                     const Aws::Crt::String &topic, 
                     const Aws::Crt::ByteBuf &payload, 
                     bool dup, 
                     Aws::Crt::Mqtt::QOS qos, 
                     bool retain);
    
    void onCloudDownChanMessageCb(Aws::Crt::Mqtt::MqttConnection &connection, 
                     const Aws::Crt::String &topic, 
                     const Aws::Crt::ByteBuf &payload, 
                     bool dup, 
                     Aws::Crt::Mqtt::QOS qos, 
                     bool retain);
    
};

}