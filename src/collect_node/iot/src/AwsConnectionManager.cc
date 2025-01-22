#include "AwsConnectionManager.h"
#include "Utils.h"
#include <boost/bind.hpp>
#include <aws/iot/MqttClient.h>
#include <aws/crt/Api.h>
#include "log.h"
namespace collect_node_iot {

AwsConnectionManager::AwsConnectionManager()
    :keepAliveTimeSecs_(10),
    pingTimeoutMs_(3*1000),
    protocolOperationTimeoutMs_(2000),
    isConnect_(false),
    connEvtIncome_(false),
    connRun_(false),
    syncConn_(false),
    connFailIntervalSec_(5) 
{
    //apiHandle_.InitializeLogging(Aws::Crt::LogLevel::Trace, "/userdata/aws_iot_sdk.log");
}

AwsConnectionManager::~AwsConnectionManager() 
{
    if (connptr_) {
        connptr_->Disconnect();
    }
    connRun_ = false;
    connCond_.notify_all();
    if (connectThread_.joinable()) {
        connectThread_.join();
    }
}

const AwsConnectionManagerPtr& AwsConnectionManager::instance()
{
    static AwsConnectionManagerPtr aws_connection_manager = 
        std::make_shared<AwsConnectionManager>();
    return aws_connection_manager;
}

bool AwsConnectionManager::isConnected() const
{
    return isConnect_.load();
}

const std::shared_ptr<Aws::Crt::Mqtt::MqttConnection>& AwsConnectionManager::getAwsConnection()
{
    return connptr_;
}

bool AwsConnectionManager::disconnect()
{
    HJ_INFO("MQTT disconnecting...\n");
    if (!connptr_) {
        HJ_INFO("mqtt not construct\n");
        return true;
    }
    
    connptr_->Disconnect();
    isConnect_.store(false);
    return true;
}

bool AwsConnectionManager::startConnect(bool sync)
{
/*
    if (isConnect_.load()) {
        HJ_INFO("MQTT already connected\n");
        return true;
    }
*/
    syncConn_ = sync;
    
    return startConnectionThread();
}

void AwsConnectionManager::startReconnect()
{
    if (isConnect_.load()) {
        std::mutex mtx;
        std::unique_lock<std::mutex> lc(mtx);
        HJ_INFO("MQTT disconnecting...\n");
        connptr_->Disconnect();
        {
            disconnCond_.wait(lc);
            HJ_INFO("MQTT disconnect!\n");
        }
    }

    HJ_INFO("MQTT connecting...\n");
    startConnectionThread();
}

void AwsConnectionManager::setEndPoint(const std::string& endpoint)
{
    endpoint_ = Aws::Crt::String(endpoint.c_str());
}

boost::signals2::connection AwsConnectionManager::addConnSuccSlot(const ConnDisConCb& slot)
{
    return conn_signal_.connect(slot);
}

boost::signals2::connection AwsConnectionManager::addConnFailSlot(const ConnFailCb& slot)
{
    return conn_fail_signal_.connect(slot);
}

boost::signals2::connection AwsConnectionManager::addDisconSlot(const ConnDisConCb& slot)
{
    return drop_signal_.connect(slot);
}

boost::signals2::connection AwsConnectionManager::addConStatListener(const ConStaChangeCb& slot)
{
    return conchange_signal_.connect(slot);
}

void AwsConnectionManager::disconnectAllConnSlots()
{
    conn_signal_.disconnect_all_slots();
    conn_fail_signal_.disconnect_all_slots();
    drop_signal_.disconnect_all_slots();
}

bool AwsConnectionManager::initialize(const std::string& cert, 
                    const std::string& key, 
                    const std::string& clientId, 
                    const std::string& endpoint) 
{
/*
    if (isConnect_.load()) {
        HJ_ERROR("Connection Manager already init!\n");
        return false;
    }
*/
    if (connRun_) {
        connRun_ = false;
        if (connectThread_.joinable()) {
            connEvtIncome_.store(true);
            HJ_INFO("iot connect notify");
            connCond_.notify_all();
            connectThread_.join();
        }
        HJ_INFO("aws conn thread joined!\n");
    }
    
    endpoint_ = Aws::Crt::String(endpoint.c_str());
    clientId_ = clientId;

    if (!mqttClientConstruct(cert, key)) {
        HJ_ERROR("Mqtt client construct fail!\n");
        return false;
    }

    connptr_->SetReconnectTimeout(2, 8);
    setMqttClientCallBack();

    if (setLastWill())
        HJ_INFO("Last Will set success!\n");
    else
        HJ_INFO("Last Will set fail!\n");

    return true;
}

bool AwsConnectionManager::mqttClientConstruct(const std::string& cert, const std::string& key)
{
    Aws::Crt::ByteCursor certIner;
    Aws::Crt::ByteCursor keyIner;
    certIner.len = cert.length();
    certIner.ptr = reinterpret_cast<uint8_t*>(const_cast<char*>(cert.data()));

    keyIner.len = key.length();
    keyIner.ptr = reinterpret_cast<uint8_t*>(const_cast<char*>(key.data()));

    auto clientConfigBuilder =
        Aws::Iot::MqttClientConnectionConfigBuilder(certIner, keyIner);

    clientConfigBuilder.WithEndpoint(endpoint_);

    // Create the MQTT connection from the MQTT builder
    auto clientConfig = clientConfigBuilder.Build();

    if (!clientConfig) {
        HJ_ERROR(
            "Client Configuration initialization failed with error %s\n",
            Aws::Crt::ErrorDebugString(clientConfig.LastError()));
        return false;
    }

    auto mqttClient = Aws::Iot::MqttClient();
    connptr_ = mqttClient.NewConnection(clientConfig);
    HJ_INFO("***mqtt new connection****\n");
    if (!*connptr_) {
        HJ_ERROR(
            "MQTT Connection Creation failed with error %s\n",
            Aws::Crt::ErrorDebugString(connptr_->LastError()));
        return false;
    }

    return true;
}

void AwsConnectionManager::setMqttClientCallBack()
{
    connptr_->OnConnectionCompleted = boost::bind(&AwsConnectionManager::connectionCompleteCb,
                                                  this,
                                                  boost::placeholders::_1, 
                                                  boost::placeholders::_2, 
                                                  boost::placeholders::_3, 
                                                  boost::placeholders::_4);
    
    connptr_->OnDisconnect = boost::bind(&AwsConnectionManager::connectingDisconnectCb,
                                         this, 
                                         boost::placeholders::_1);

    connptr_->OnConnectionInterrupted = boost::bind(&AwsConnectionManager::connectionInterruptCb,
                                                    this, 
                                                    boost::placeholders::_1,
                                                    boost::placeholders::_2);
    
    connptr_->OnConnectionResumed = boost::bind(&AwsConnectionManager::connectionResumedCb,
                                                this,
                                                boost::placeholders::_1,
                                                boost::placeholders::_2,
                                                boost::placeholders::_3);

    connptr_->OnConnectionSuccess = boost::bind(&AwsConnectionManager::connectionSuccessCb,
                                                this,
                                                boost::placeholders::_1,
                                                boost::placeholders::_2);

    connptr_->OnConnectionFailure = boost::bind(&AwsConnectionManager::connectionFalureCb,
                                                this,
                                                boost::placeholders::_1,
                                                boost::placeholders::_2);
}

bool AwsConnectionManager::startConnectionThread()
{
    if (connRun_) {
        connEvtIncome_.store(true);
        connRun_ = false;
        HJ_INFO("iot connect notify");
        connCond_.notify_all();
    }

    if (connectThread_.joinable())
        connectThread_.join();

    isConnect_.store(false);
    connectThread_ = std::thread(&AwsConnectionManager::connectThreadFunc, this);
    
    if (syncConn_) {
        if (connectThread_.joinable()) {
            connectThread_.join();
        }
        return isConnect_.load();
    } else {
        return true;
    }
}

bool AwsConnectionManager::setLastWill()
{
    std::string topic;
    topic = "aiper/things/" + clientId_ + "/willChan";
    std::string dymsg = "{\"type\":\"WillMsgReport\", \"data\":{\"online\":0}}";
    Aws::Crt::ByteBuf buf = Aws::Crt::ByteBufFromArray((const uint8_t *)dymsg.data(), dymsg.length());
    return connptr_->SetWill(topic.data(), AWS_MQTT_QOS_AT_MOST_ONCE, false, buf);
}

void AwsConnectionManager::connectionCompleteCb(
                          Aws::Crt::Mqtt::MqttConnection&,
                          int errorCode,
                          Aws::Crt::Mqtt::ReturnCode returnCode,
                          bool) 
{
    if (errorCode) {
        HJ_ERROR("Connection ack failed with error %s\n", Aws::Crt::ErrorDebugString(errorCode));
    } else {
        HJ_INFO("Connection ack succ with return code %d\n", returnCode);
    }
}

void AwsConnectionManager::connectionSuccessCb(
                          Aws::Crt::Mqtt::MqttConnection &, 
                          Aws::Crt::Mqtt::OnConnectionSuccessData* succdata)
{
    HJ_INFO("MQTT Connection success:%d\n", succdata->returnCode);
    isConnect_.store(true);
    conn_signal_();
    connEvtIncome_.store(true);
    HJ_INFO("iot connect notify");
    connCond_.notify_one();
}

void AwsConnectionManager::connectionFalureCb(
                          Aws::Crt::Mqtt::MqttConnection &,
                          Aws::Crt::Mqtt::OnConnectionFailureData* faildata)
{
    HJ_ERROR("MQTT Connection fail:%s\n", Aws::Crt::ErrorDebugString(faildata->error));
    isConnect_.store(false);
    conn_fail_signal_(faildata->error, Aws::Crt::ErrorDebugString(faildata->error));
    connEvtIncome_.store(true);
    HJ_INFO("iot connect notify");
    connCond_.notify_one();
}

void AwsConnectionManager::connectingDisconnectCb(
                          Aws::Crt::Mqtt::MqttConnection&)
{
    HJ_INFO("MQTT Disconnect completed\n");
    isConnect_.store(false);
    drop_signal_();
    conchange_signal_(false);
    disconnCond_.notify_all();
}

void AwsConnectionManager::connectionInterruptCb(
                          Aws::Crt::Mqtt::MqttConnection&,
                          int)
{
    HJ_INFO("MQTT Connection lost!\n");
    conchange_signal_(false);
    isConnect_.store(false);
}

void AwsConnectionManager::connectionResumedCb(Aws::Crt::Mqtt::MqttConnection&,
                          Aws::Crt::Mqtt::ReturnCode returnCode,
                          bool sessionPresent)
{
    HJ_INFO("MQTT Connection resumed, session present:%d", sessionPresent);
    if (!sessionPresent) {
        if (sessionResumeCb_) {
            sessionResumeCb_(false);
        }
        return;
    }
    conchange_signal_(true);
    isConnect_.store(true);
}

void AwsConnectionManager::connectThreadFunc()
{
    connRun_ = true;
    int failtms = 0;

    while (!isConnect_.load() && connRun_) {
        std::unique_lock<std::mutex> lc(connmtx_);
        connEvtIncome_.store(false);
        HJ_INFO("MQTT Connection [%s] start...\n", clientId_.c_str());
        bool connRst = connptr_->Connect(clientId_.data(), false, keepAliveTimeSecs_, pingTimeoutMs_, protocolOperationTimeoutMs_);
        HJ_INFO("MQTT Connection result: %d\n", connRst);
        if (connRst) {
            HJ_INFO("iot connect wait...");
            connCond_.wait(lc, [&]{
                return connEvtIncome_.load();
            });
            HJ_INFO("iot connect wait done");
        }

        if (!connRun_) {
            break;
        }

        if (!isConnect_.load()) { 
            HJ_ERROR("MQTT Connection failed with error %s\n", Aws::Crt::ErrorDebugString(connptr_->LastError()));
            if (syncConn_) {
                failtms++;
                if (failtms >= 2) {
                    HJ_ERROR("sync conn fail\n");
                    break;
                }
            }
            sleep(connFailIntervalSec_);  
        }
    }

    connRun_ = false;
    HJ_INFO("MQTT Connection exit, ret=%d\n", isConnect_.load());
}

}
