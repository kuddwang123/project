/**
 * @ Author: lou.pan
 * @ Create Time: 2024-03-27 17:58:30
 * @ Modified by: lou.pan
 * @ Modified time: 2024-05-22 11:05:02
 * @ Description: Manage AWS core connection
 */

#pragma once

#include <string>
#include <memory>
#include <aws/crt/mqtt/MqttConnection.h>
#include <aws/crt/Types.h>
#include <aws/crt/Api.h>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <boost/signals2.hpp>
#include "Defines.h"

namespace collect_node_iot {
class AwsConnectionManager;
typedef std::shared_ptr<AwsConnectionManager> AwsConnectionManagerPtr;

class AwsConnectionManager
{
public:
    static const AwsConnectionManagerPtr& instance();
    AwsConnectionManager();
    ~AwsConnectionManager();

  //连接初始化，设置参数和连接回调
    bool initialize(const std::string& cert, 
                    const std::string& key, 
                    const std::string& clientId, 
                    const std::string& endpoint);
  
  //获取Aws Mqtt连接实例对象
    const std::shared_ptr<Aws::Crt::Mqtt::MqttConnection>& getAwsConnection();
  
  //设置终端设备号
    void setEndPoint(const std::string&);
  
  //获取连接状态
    bool isConnected() const;
  
  //断开Aws连接
    void disconnect();
  
  //开始Aws连接
    void startConnect();
  
  //开始Aws重连
    void startReconnect();
  
  //注册Aws连接成功回调
    boost::signals2::connection addConnSuccSlot(const ConnDisConCb& slot);
  
  //注册Aws连接失败回调
    boost::signals2::connection addConnFailSlot(const ConnFailCb& slot);

  //注册Aws断连成功回调
    boost::signals2::connection addDisconSlot(const ConnDisConCb& slot);
  
  //注册Aws连接断开/恢复回调
    boost::signals2::connection addConStatListener(const ConStaChangeCb& slot);
  
private:
    std::string clientId_;
    Aws::Crt::String endpoint_;
    Aws::Crt::ApiHandle apiHandle_;
    std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> connptr_;
    uint16_t keepAliveTimeSecs_;
    uint32_t pingTimeoutMs_;
    uint32_t protocolOperationTimeoutMs_;
    std::atomic<bool> isConnect_;
    bool connRun_;
    int connFailIntervalSec_;
    std::thread connectThread_;

    std::condition_variable connCond_;
    std::condition_variable disconnCond_;

    boost::signals2::signal<void()> conn_signal_;
    boost::signals2::signal<void(int, std::string)> conn_fail_signal_;
    boost::signals2::signal<void()> drop_signal_;
    boost::signals2::signal<void(bool)> conchange_signal_;

private:
    void startConnectionThread();

    void connectThreadFunc();

    bool mqttClientConstruct(const std::string& cert, const std::string& key);

    void setMqttClientCallBack();

    // Invoked when a MQTT connect has completed or error occured
    void connectionCompleteCb(Aws::Crt::Mqtt::MqttConnection&,
                              int,
                              Aws::Crt::Mqtt::ReturnCode,
                              bool);
    
    // Invoked when a disconnect message has completed.
    void connectingDisconnectCb(Aws::Crt::Mqtt::MqttConnection&);

    // Invoked when a MQTT connection succeed
    void connectionSuccessCb(Aws::Crt::Mqtt::MqttConnection &, 
                             Aws::Crt::Mqtt::OnConnectionSuccessData*);
    
    // Invoked when a MQTT connection fails
    void connectionFalureCb(Aws::Crt::Mqtt::MqttConnection &,
                            Aws::Crt::Mqtt::OnConnectionFailureData*);
    
    // Invoked when a MQTT connection was interrupted/lost
    void connectionInterruptCb(Aws::Crt::Mqtt::MqttConnection&,
                               int);
    
    // Invoked when a MQTT connection resumed.
    void connectionResumedCb(Aws::Crt::Mqtt::MqttConnection&,
                             Aws::Crt::Mqtt::ReturnCode,
                             bool);
    // Set Last Will
    bool setLastWill();
};
}