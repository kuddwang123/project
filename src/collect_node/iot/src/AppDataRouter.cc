#include "AppDataRouter.h"
#include "log.h"
#include "error.h"
#include <algorithm>
#include <aws/crt/UUID.h>
#include "Utils.h"
#include "Persistence.h"
#include <std_msgs/UInt8.h>
#include <hj_interface/AppOnlineType.h>
namespace collect_node_iot {

utils::CountDownLatch cdlatch_g(2);

AppDataRouter::AppDataRouter(const std::string& certfile):
    awsConnectionPtr_(AwsConnectionManager::instance()),
    btConnectionPtr_(BtConnectionManager::instance()),
    apConnectionPtr_(ApNetConfigManger::instance()),
    shadowClientLinkPtr_(std::make_shared<IotShadowClientLink>()),
    iotMqttClientLinkPtr_(std::make_shared<IotMqttClientLink>()),
    iotRun_(false),
    certfile_(certfile)
{
    assert(awsConnectionPtr_);
    assert(btConnectionPtr_);
    assert(apConnectionPtr_);
    assert(shadowClientLinkPtr_);
    assert(iotMqttClientLinkPtr_);
}

AppDataRouter::~AppDataRouter()
{

}

void AppDataRouter::initialize(uint16_t port)
{
    apConnectionPtr_->setApDataHandler(boost::bind(&AppDataRouter::apMsgIncomeEvent, this,
                                boost::placeholders::_1,
                                boost::placeholders::_2,
                                boost::placeholders::_3));

    apConnectionPtr_->startSrvLoop(port);

    btConnectionPtr_->initialize();
    btConnectionPtr_->setBtDataHandler(boost::bind(&AppDataRouter::btMsgIncomeEvent, this,
                                boost::placeholders::_1,
                                boost::placeholders::_2));

    btConnectionPtr_->addConStatListener(boost::bind(&AppDataRouter::btConnStaChangeSlot, this,
                                boost::placeholders::_1));

    awsConnectionPtr_->addConStatListener(boost::bind(&AppDataRouter::awsConnStaChangeSlot, this,
                                boost::placeholders::_1));
    
    awsConnectionPtr_->setSessionDealCb(boost::bind(&AppDataRouter::dealSessionFunc, this,
                                boost::placeholders::_1));

    Persistence::instance().initialize(shared_from_this());

    initRos();
}


bool AppDataRouter::initIot(
        const std::string& cert, 
        const std::string& key, 
        const std::string& clientId, 
        const std::string& endPoint, 
        const std::string& thingName)
{
/*
    if (awsConnectionPtr_->isConnected()) {
       awsConnectionPtr_->disconnect();
       sleep(2);
    }
*/
    if (!awsConnectionPtr_->initialize(cert, key, clientId, endPoint)) {
        HJ_ERROR("aws connection init fail\n");
        return false;
    }

    return constructAwsMqttConn(thingName);
}

bool AppDataRouter::restartIot(bool flag)
{
    if (!shadowClientLinkPtr_ || !iotMqttClientLinkPtr_ || !awsConnectionPtr_) {
        HJ_ERROR("MQTT client not construct!\n");
        return false;
    }

    utils::AwsCertParser awsCertParser(certfile_.data());
    if (!awsCertParser.parseOk()) {
        HJ_ERROR("cert parse error\n");
        return false;
    }

    std::lock_guard<std::mutex> lc(constructIotMtx_);
    if (!awsConnectionPtr_->initialize(awsCertParser.cert(), 
                                       awsCertParser.prikey(), 
                                       awsCertParser.thingname(), 
                                       awsCertParser.endpoint())) {
        return false;
    }
    
    if (!constructAwsMqttConn(awsCertParser.thingname())) {
        return false;
    }

    return runIot(flag);
}

void AppDataRouter::dealSessionFunc(bool sessionPresent)
{
    HJ_INFO("deal session: %d\n", sessionPresent);
    if (!sessionPresent) {
        this->restartIot();
    }
}

bool AppDataRouter::constructAwsMqttConn(const std::string& thing)
{
    auto shadowClient = std::make_shared<IotShadowClient>(thing);
    assert(shadowClient);

    if (!shadowClientLinkPtr_->initialize(shadowClient, 
                    boost::bind(&AppDataRouter::deltaUpdateEvent, this, boost::placeholders::_1))) {
        HJ_ERROR("shadow client link init fail\n");
        return false;
    }
    shadowClientLinkPtr_->setShadowUpdateAcceptCb(boost::bind(&AppDataRouter::shadowUpdateAccptCb, this,
            boost::placeholders::_1));

    auto mqttClient = std::make_shared<IotMqttClient>(thing);
    assert(mqttClient);

    if (!iotMqttClientLinkPtr_->initialize(mqttClient)) {
        HJ_ERROR("mqtt client link init fail\n");
        return false;
    }
    iotMqttClientLinkPtr_->setMqttReqCb(boost::bind(&AppDataRouter::mqttMsgIncomeEvent, this, boost::placeholders::_1));

    awsConnectionPtr_->disconnectAllConnSlots();
    awsConnectionPtr_->addConnSuccSlot(boost::bind(&IotShadowClient::connectSlot, shadowClient));
    awsConnectionPtr_->addDisconSlot(boost::bind(&IotShadowClient::disconnSlot, shadowClient));
    
    awsConnectionPtr_->addConnSuccSlot(boost::bind(&IotMqttClient::connectSlot, mqttClient));
    awsConnectionPtr_->addDisconSlot(boost::bind(&IotMqttClient::disconnSlot, mqttClient));

    awsConnectionPtr_->addConnFailSlot(boost::bind(&AppDataRouter::connFailSlot, this, 
            boost::placeholders::_1, boost::placeholders::_2));
    
    return true;
}

bool AppDataRouter::runIot(bool sync)
{
    if (!shadowClientLinkPtr_ || !iotMqttClientLinkPtr_ || !awsConnectionPtr_) {
        HJ_ERROR("MQTT client not construct!\n");
        return false;
    }

    cdlatch_g.resetCount(2);
    iotRun_ = true;

    if (!awsConnectionPtr_->startConnect(sync)) {
        HJ_ERROR("start conn fail\n");
        return false;
    }


    if (!sync) {
        std::thread th([&]() {
            this->iotSubscribeDone(true);
        });
        th.detach();
        return true;
    }

    return iotSubscribeDone(false);
}

bool AppDataRouter::iotSubscribeDone(bool flag)
{
    if (!cdlatch_g.await(flag)) {
        HJ_ERROR("conn await fail\n");
        cdlatch_g.resetCount(2);
        return false;
    }

    cdlatch_g.resetCount(2);
    HJ_INFO("conn await success\n");
    this->connectSlot();
    this->awsConnStaChangeSlot(true);

    return true;
}

bool AppDataRouter::stopIot(bool force)
{
    iotRun_ = false;
    /*
    if (force) {
        if (awsConnectionPtr_) {
            cloudNotifyOffline();
            updateShadowOffline();
            sleep(1);
            awsConnectionPtr_->disconnect();
        }
    } else 
    */if (awsConnectionPtr_ && awsConnectionPtr_->isConnected()) {
        cloudNotifyOffline();
        updateShadowOffline();
        sleep(1);
        awsConnectionPtr_->disconnect();
    }
    
    return true;
}

void AppDataRouter::unbindIot()
{
    HJ_INFO("unbind iot...\n");
    iotRun_ = false;
    awsConnectionPtr_->disconnect();
}

void AppDataRouter::initRos()
{
    getShadowDoc_ = hj_bf::HJCreateServer("/IotGetShadow", &AppDataRouter::getShadow, this);
    
    deleteShadowDoc_ = hj_bf::HJCreateServer("/IotDeleteShadow", &AppDataRouter::deleteShadow, this);
    
    iotServer_ = hj_bf::HJCreateServer("/IotUpdateShadow", &AppDataRouter::iotUpdateShadow, this);

    appDataPub_ = hj_bf::HJAdvertise<hj_interface::AppMsg>("/ReqFromApp", 10);

    iotStatPub_ = hj_bf::HJAdvertise<hj_interface::AppOnlineType>("/AppOnline", 1, true);

    deviceRspApp_ = hj_bf::HJSubscribe("/RespToApp", 50, &AppDataRouter::deviceRespCallBack, this);

    deviceReport_ = hj_bf::HJSubscribe("/ReportApp", 50, &AppDataRouter::deviceReportCallBack, this);

    mqttTmr_ = hj_bf::HJCreateTimer("initMqtt", 5 * 1000 * 1000, &AppDataRouter::mqttInitTmrCb, this, false);
}

void AppDataRouter::connectSlot()
{
    HJ_INFO("iot conn success, update online\n");
    //mqttTmr_.start();
    rapidjson::Document document;
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;

    document.SetObject();
    //document.AddMember("ble", 2, document.GetAllocator());
    //document.AddMember("ap", 0, document.GetAllocator());
    //document.AddMember("sta", 2, document.GetAllocator());
    document.AddMember("cert", 1, document.GetAllocator());
    document.AddMember("online", 1, document.GetAllocator());
    
    appdata.key = "NetStatReport";
    appdata.payload = utils::documentToString(document);
    appmsg.appdata.push_back(appdata);
    iotMqttClientLinkPtr_->iotMqttReport(appmsg, hj_interface::AppMsg::TOPIC);
}

void AppDataRouter::connFailSlot(int code, std::string msg)
{
    HJ_ERROR("iot conn fail: %d, %s\n", code, msg.c_str());
}

void AppDataRouter::awsConnStaChangeSlot(bool state)
{
    if (!awsConnectionPtr_ || !btConnectionPtr_)
        return;

    hj_interface::AppOnlineType msg;

    if (state) {
        updateShadowOnline();
        if (btConnectionPtr_->isConnected()) 
            msg.type = hj_interface::AppOnlineType::BT_IOT;
        else
            msg.type = hj_interface::AppOnlineType::IOT;
    } else {
        if (btConnectionPtr_->isConnected()) 
            msg.type = hj_interface::AppOnlineType::BT;
        else
            msg.type = hj_interface::AppOnlineType::OFFLINE;
    }

    HJ_INFO("aws conn state change to %d, pub:%d\n", state, msg.type);
    iotStatPub_.publish(msg);
}

void AppDataRouter::btConnStaChangeSlot(bool state)
{
    if (!awsConnectionPtr_ || !btConnectionPtr_)
        return;

    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    rapidjson::Document reportDoc;
    reportDoc.SetObject();
    reportDoc.AddMember("ble", state ? 1 : 2, reportDoc.GetAllocator());
    appdata.key = "NetStat";
    appdata.payload = utils::documentToString(reportDoc);
    appmsg.appdata.push_back(appdata);
    shadowClientLinkPtr_->dvcUpdateShadow(appmsg);

    hj_interface::AppOnlineType msg;

    if (state) {
        if (awsConnectionPtr_->isConnected())
            msg.type = hj_interface::AppOnlineType::BT_IOT;
        else
            msg.type = hj_interface::AppOnlineType::BT;
    } else {
        if (awsConnectionPtr_->isConnected()) 
            msg.type  = hj_interface::AppOnlineType::IOT;
        else
            msg.type = hj_interface::AppOnlineType::OFFLINE;
    }

    HJ_INFO("bt conn state change to %d, pub:%d\n", state, msg.type);
    iotStatPub_.publish(msg);
}

void AppDataRouter::updateShadowOnline()
{
    HJ_INFO("update shadow online!\n");
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    
    appdata.key = "NetStat";
    appdata.payload = "{\"online\":1}";
    appmsg.appdata.push_back(appdata);
    shadowClientLinkPtr_->dvcUpdateShadow(appmsg);
}

void AppDataRouter::updateShadowOffline()
{
    HJ_INFO("update shadow offline!\n");
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    
    appdata.key = "NetStat";
    appdata.payload = "{\"online\":0}";
    appmsg.appdata.push_back(appdata);
    shadowClientLinkPtr_->dvcUpdateShadow(appmsg);
}

void AppDataRouter::cloudNotifyOffline()
{
    HJ_INFO("notify cloud offline!\n");
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    
    appdata.key = "NetStatReport";
    appdata.payload = "{\"online\":0}";
    appmsg.to = hj_interface::AppMsg::CLOUD;
    appmsg.appdata.push_back(appdata);
    doAppMsgReport(appmsg);
}

void AppDataRouter::mqttInitTmrCb(const hj_bf::HJTimerEvent&)
{
    //iotMqttClientLinkPtr_->setMqttReqCb(boost::bind(&AppDataRouter::mqttMsgIncomeEvent, this, boost::placeholders::_1));
    //mqttTmr_.stop();
}

void AppDataRouter::shadowUpdateAccptCb(const std::vector<hj_interface::AppData>& data)
{
    for (const auto& app: data) {
        HJ_INFO("shadow update success: %s %s", app.key.c_str(), app.payload.c_str());
        if (app.key == "NetStat" && iotRun_) {
            rapidjson::Document doc;
            if (doc.Parse(app.payload.data()).HasParseError()) {
                continue;
            }
            if (doc.HasMember("online") && doc["online"].IsInt()) {
                int online = doc["online"].GetInt();
                if (online == 0) {
                    HJ_ERROR("shadow online update 0!");
                    updateShadowOnline();
                }
            }
        }
    }
}

void AppDataRouter::deviceReportCallBack(const hj_interface::AppMsg::ConstPtr& msg)
{
    HJ_INFO("receive report to app, size:%ld, to [%d]\n", msg->appdata.size(), msg->to);

    if (msg->appdata.empty()) {
        HJ_ERROR("report app data null\n");
        return;
    }
    
    hj_interface::AppMsg appmsg = *msg.get();

    for (auto it = appmsg.appdata.begin(); it != appmsg.appdata.end();) {
        HJ_INFO("[%s]\n[%s]\n", it->key.c_str(), it->payload.c_str());
        if (Persistence::instance().needPersistent(it->key)) {
            Persistence::instance().persistMsg(it->key, it->payload);
            it = appmsg.appdata.erase(it);
        } else {
            ++it;
        }
    }

    if (!appmsg.appdata.empty()) {
        doAppMsgReport(appmsg);
    }
    
    return;
}

void AppDataRouter::deviceRespCallBack(const hj_interface::AppMsg::ConstPtr& msg)
{
    HJ_INFO("receive response to app, from [%d]\n", msg->from);
    if (msg->appdata.empty()) {
        HJ_ERROR("response data empty\n");
        return;
    }

    auto appdata = msg->appdata.at(0);
    HJ_INFO("[%s]\n[%s]\n", appdata.key.c_str(),appdata.payload.c_str());

    if (msg->from == IOT_TOPIC || msg->from == IOT_CLOUD_TOPIC) {
        if (!iotMqttClientLinkPtr_)
            return;
        iotMqttClientLinkPtr_->iotMqttResp(msg);
    } else if (msg->from == BLUETOOTH) {
        if (msg->appdata.size() > 1) {
            HJ_ERROR("response size:%ld not support via bluetooth\n", msg->appdata.size());
            return;
        }

        // {"key":{}, "res":xxx}
        btConnectionPtr_->btSend(msg->appdata.at(0).key, 
                msg->appdata.at(0).payload,
                msg->appdata.at(0).res);
    } else if (msg->from == WIFI) {
        if (msg->appdata.size() > 1) {
            HJ_ERROR("response size:%ld not support via ap\n", msg->appdata.size());
            return;
        }

        // {"key":{}, "res":xxx}
        apConnectionPtr_->apSend(msg->appdata.at(0).key,
                msg->appdata.at(0).payload,
                msg->appdata.at(0).res,
                msg->session
                );
    }
}

void AppDataRouter::doAppMsgReport(const hj_interface::AppMsg& msg)
{
    if (!awsConnectionPtr_->isConnected() && (msg.to != hj_interface::AppMsg::BLUETOOTH &&
            msg.to != hj_interface::AppMsg::AP)) {
        return;
    }

    if (msg.to & hj_interface::AppMsg::TOPIC) {
        if (iotMqttClientLinkPtr_)
            iotMqttClientLinkPtr_->iotMqttReport(msg, hj_interface::AppMsg::TOPIC);
    }
    
    if (msg.to & hj_interface::AppMsg::SHADOW) {
        if (shadowClientLinkPtr_)
            shadowClientLinkPtr_->dvcUpdateShadow(msg);
    }
    
    if (msg.to & hj_interface::AppMsg::CLOUD) {
        if (iotMqttClientLinkPtr_)
            iotMqttClientLinkPtr_->iotMqttReport(msg, hj_interface::AppMsg::CLOUD);
    }
    
    if (msg.to & hj_interface::AppMsg::BIGDATA) {
        if (iotMqttClientLinkPtr_)
            iotMqttClientLinkPtr_->iotMqttReport(msg, hj_interface::AppMsg::BIGDATA);
    }

    if ((msg.to & hj_interface::AppMsg::BLUETOOTH) || btConnectionPtr_->isConnected()) {
        if (msg.appdata.size() > 1) {
            HJ_ERROR("report size:%ld not support via bluetooth\n", msg.appdata.size());
            return;
        }

        if (msg.to != hj_interface::AppMsg::BIGDATA)
            btConnectionPtr_->btRpt(msg.appdata.at(0).key, msg.appdata.at(0).payload);
    }

    if (msg.to & hj_interface::AppMsg::AP) {
        apConnectionPtr_->apReport(msg.appdata.at(0).key, msg.appdata.at(0).payload, msg.session);
    }
}

void AppDataRouter::appDataPub(const hj_interface::AppMsg& appmsg)
{
    if(appDataAnalyseCb_) {
        if (appDataAnalyseCb_(appmsg)) {
            return;
        }
    }
    appDataPub_.publish(appmsg);
}

void AppDataRouter::deltaUpdateEvent(hj_interface::AppMsg& data)
{
    HJ_INFO("delta msg income:\n");

    for (const auto& app: data.appdata) {
        HJ_INFO("\n[%s]\n%s\n", app.key.c_str(), app.payload.c_str());
    }

    data.from = IOT_SHADOW;

    appDataPub(data);
}

void AppDataRouter::mqttMsgIncomeEvent(const hj_interface::AppMsg& data)
{
    HJ_INFO("mqtt msg income:\n");

    for (const auto& app: data.appdata) {
        HJ_INFO("\n[%s]\n%s\n", app.key.c_str(), app.payload.c_str());
    }

    appDataPub(data);
}

void AppDataRouter::btMsgIncomeEvent(const std::string& key, const std::string& payload)
{   
    HJ_INFO("bt msg income:%s, %s\n", key.data(), payload.data()); 

    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = key;
    appdata.payload = payload;

    appmsg.appdata.emplace_back(appdata);
    appmsg.from = BLUETOOTH;

    appDataPub(appmsg);
}

void AppDataRouter::apMsgIncomeEvent(const std::string& key, const std::string& payload, const std::string& sessId)
{
    HJ_INFO("ap msg income:%s, %s\n", key.data(), payload.data()); 

    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = key;
    appdata.payload = payload;

    appmsg.appdata.emplace_back(appdata);
    appmsg.from = WIFI;
    appmsg.session = sessId;

    appDataPub(appmsg);
}

void AppDataRouter::sendAppResp(const hj_interface::AppMsg& msg)
{
    deviceRespCallBack(boost::make_shared<hj_interface::AppMsg>(msg));
}

void AppDataRouter::sendAppRpt(const hj_interface::AppMsg& msg)
{
    deviceReportCallBack(boost::make_shared<hj_interface::AppMsg>(msg));
}

bool AppDataRouter::getShadow(hj_interface::IotShadowRequest& req,
                   hj_interface::IotShadowResponse& res)
{
    if (!shadowClientLinkPtr_) {
        res.iotret.code = IOT_CONNECTION_LOST;
        res.iotret.msg = getMessageByCode(IOT_CONNECTION_LOST);
        return true;
    }

    return shadowClientLinkPtr_->linkGetShadow(req, res);
}

bool AppDataRouter::deleteShadow(hj_interface::IotShadowRequest& req,
                      hj_interface::IotShadowResponse& res)
{
    if (!shadowClientLinkPtr_) {
        res.iotret.code = IOT_CONNECTION_LOST;
        res.iotret.msg = getMessageByCode(IOT_CONNECTION_LOST);
        return true;
    }

    return shadowClientLinkPtr_->linkDeleteShadow(req, res);
}
    
bool AppDataRouter::iotUpdateShadow(hj_interface::IotShadowRequest& req,
                    hj_interface::IotShadowResponse& res)
{
    if (!shadowClientLinkPtr_) {
        res.iotret.code = IOT_CONNECTION_LOST;
        res.iotret.msg = getMessageByCode(IOT_CONNECTION_LOST);
        return true;
    }

    return shadowClientLinkPtr_->linkIotUpdateShadow(req, res);
}

}