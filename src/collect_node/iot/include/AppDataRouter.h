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
#include <memory>
#include <mutex>
namespace collect_node_iot {
class AppDataRouter;
class Persistence;

typedef std::shared_ptr<AppDataRouter> AppDataRouterPtr;

class AppDataRouter: public std::enable_shared_from_this<AppDataRouter> {
typedef boost::function<bool(const hj_interface::AppMsg&)> appDataAnalyseCb;

public:
    AppDataRouter(const std::string& certfile);
    ~AppDataRouter();

  //初始化，实例化iot连接，bt连接, tcp服务和初始化ros
    void initialize(uint16_t port);

  //Aws Iot连接初始化
    bool initIot(const std::string& cert, const std::string& key, 
       const std::string& clientId, const std::string& endPoint, const std::string& thingName);
  
  //Aws Iot启动运行
    bool runIot(bool);

  //Aws Iot断开连接
    bool stopIot(bool force = false);
  
  //APP Iot解绑处理
    void unbindIot();
  
  //构造MQTT连接
    bool constructAwsMqttConn(const std::string& thing);

  //Aws Iot重连
    bool restartIot();

  //注册蓝牙数据处理
    void setAppDataAnalyser(const appDataAnalyseCb cb) { appDataAnalyseCb_ = cb; }

  //发送响应数据
    void sendAppResp(const hj_interface::AppMsg&);

  //发送上报数据
    void sendAppRpt(const hj_interface::AppMsg&);

    friend class Persistence;

private:
    hj_bf::HJServer getShadowDoc_;
    hj_bf::HJServer deleteShadowDoc_;
    hj_bf::HJServer iotServer_;
    hj_bf::HJPublisher appDataPub_;
    hj_bf::HJPublisher iotStatPub_;
    hj_bf::HJSubscriber deviceReport_;
    hj_bf::HJSubscriber deviceRspApp_;
    hj_bf::HJTimer mqttTmr_;

    AwsConnectionManagerPtr awsConnectionPtr_;
    BtConnectionManagerPtr btConnectionPtr_;
    ApNetConfigManagerPtr apConnectionPtr_;
    std::shared_ptr<IotShadowClientLink> shadowClientLinkPtr_;
    std::shared_ptr<IotMqttClientLink> iotMqttClientLinkPtr_;
    
    appDataAnalyseCb appDataAnalyseCb_;
    bool iotRun_;
    std::string certfile_;
    std::mutex constructIotMtx_;

private:
    void connectSlot();

    void connFailSlot(int, std::string);
    
    void awsConnStaChangeSlot(bool);

    void btConnStaChangeSlot(bool);

    void initRos();

    bool iotSubscribeDone(bool);

    void updateShadowOnline();

    void updateShadowOffline();

    void dealSessionFunc(bool sessionPresent);

    void doAppMsgReport(const hj_interface::AppMsg&);

    void mqttInitTmrCb(const hj_bf::HJTimerEvent&);
    
    void shadowUpdateAccptCb(const std::vector<hj_interface::AppData>&);

    void appDataPub(const hj_interface::AppMsg&);

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