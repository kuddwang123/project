/**
 * @ Author: lou.pan
 * @ Create Time: 2024-03-27 17:59:22
 * @ Modified by: lou.pan
 * @ Modified time: 2024-06-04 15:32:30
 * @ Description: Handle data transport between device and App
 */
#pragma once

#include "node_factory.h"
#include "hj_interface/IotShadow.h"
#include "hj_interface/AppMsg.h"
#include "IotShadowClientLink.h"
#include "IotMqttClientLink.h"
#include "BtConnectionManager.h"
#include "ApConnectionManager.h"

namespace collect_node_iot {

class AppDataRouter {
typedef boost::function<bool(const hj_interface::AppMsg&)> appDataAnalyseCb;

public:
    AppDataRouter();
    ~AppDataRouter();

  //初始化，实例化iot连接，bt连接, tcp服务和初始化ros
    void initialize(uint16_t port);

  //Aws Iot连接初始化
    bool initIot(const std::string& cert, const std::string& key, 
       const std::string& clientId, const std::string& endPoint, const std::string& thingName);
  
  //Aws Iot启动运行
    bool runIot();

  //Aws Iot断开连接
    bool stopIot();
    
  //注册蓝牙数据处理
    void setAppDataAnalyser(const appDataAnalyseCb cb) { appDataAnalyseCb_ = cb; }

  //发送响应数据
    void sendAppResp(const hj_interface::AppMsg&);

  //发送上报数据
    void sendAppRpt(const hj_interface::AppMsg&);

private:
    hj_bf::HJServer getShadowDoc_;
    hj_bf::HJServer deleteShadowDoc_;
    hj_bf::HJServer iotServer_;
    hj_bf::HJPublisher appDataPub_;
    hj_bf::HJPublisher iotStatPub_;
    hj_bf::HJSubscriber deviceReport_;
    hj_bf::HJSubscriber deviceRspApp_;

    AwsConnectionManagerPtr awsConnectionPtr_;
    BtConnectionManagerPtr btConnectionPtr_;
    ApNetConfigManagerPtr apConnectionPtr_;
    std::shared_ptr<IotShadowClientLink> shadowClientLinkPtr_;
    std::shared_ptr<IotMqttClientLink> iotMqttClientLinkPtr_;
    
    appDataAnalyseCb appDataAnalyseCb_;

private:
    void connectSlot();

    void connFailSlot(int, std::string);
    
    void awsConnStaChangeSlot(bool);

    void btConnStaChangeSlot(bool);

    void initRos();

    void updateShadowOnline();

    bool getShadow(hj_interface::IotShadowRequest&,
                   hj_interface::IotShadowResponse&);
    
    bool deleteShadow(hj_interface::IotShadowRequest&,
                      hj_interface::IotShadowResponse&);
    
    bool iotUpdateShadow(hj_interface::IotShadowRequest&,
                    hj_interface::IotShadowResponse&);
    
    void deltaUpdateEvent(hj_interface::AppMsg&);

    void mqttMsgIncomeEvent(const hj_interface::AppMsg&);

    void deviceReportCallBack(const hj_interface::AppMsg::ConstPtr&);

    void deviceRespCallBack(const hj_interface::AppMsg::ConstPtr&);

    void btMsgIncomeEvent(const std::string&, const std::string&);

    void apMsgIncomeEvent(const std::string&, const std::string&, const std::string&);
};

}