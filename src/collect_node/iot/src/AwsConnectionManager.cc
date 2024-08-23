#include "AwsConnectionManager.h"
#include <boost/bind.hpp>
#include <aws/iot/MqttClient.h>
#include <aws/crt/Api.h>
#include <mutex>

#include "log.h"
namespace collect_node_iot {

AwsConnectionManager::AwsConnectionManager()
    :keepAliveTimeSecs_(20),
    pingTimeoutMs_(15*1000),
    protocolOperationTimeoutMs_(2000),
    isConnect_(false),
    connRun_(false),
    connFailIntervalSec_(5) 
{
}

AwsConnectionManager::~AwsConnectionManager() 
{
    connptr_->Disconnect();
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

void AwsConnectionManager::disconnect()
{
    if(!isConnect_.load()) {
        HJ_INFO("MQTT already disconnected!\n");
        return;
    }

    std::mutex mtx;
    std::unique_lock<std::mutex> lc(mtx);
    HJ_INFO("MQTT disconnecting...\n");
    connptr_->Disconnect();
    /*
    {
        disconnCond_.wait(lc);
        HJ_INFO("MQTT disconnect!\n");
    }*/
}

void AwsConnectionManager::startConnect()
{
    if (isConnect_.load()) {
        HJ_INFO("MQTT already connected, start reconnect\n");
        startReconnect();
        return;
    }
    
    startConnectionThread();
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

bool AwsConnectionManager::initialize(const std::string& cert, 
                    const std::string& key, 
                    const std::string& clientId, 
                    const std::string& endpoint) 
{
    if (isConnect_.load()) {
        HJ_ERROR("Connection Manager already init!\n");
        return false;
    }

    if (connRun_) {
        connRun_ = false;
        if (connectThread_.joinable())
            connectThread_.join();
        HJ_INFO("aws conn thread joined!\n");
    }
    
    endpoint_ = Aws::Crt::String(endpoint.c_str());
    clientId_ = clientId;

    if (!mqttClientConstruct(cert, key)) {
        HJ_ERROR("Mqtt client construct fail!\n");
        return false;
    }

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

void AwsConnectionManager::startConnectionThread()
{
    if (connRun_)
        return;

    if (connectThread_.joinable())
        connectThread_.join();

    connectThread_ = std::thread(&AwsConnectionManager::connectThreadFunc, this);
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
        HJ_ERROR("Connection failed with error %s\n", Aws::Crt::ErrorDebugString(errorCode));
        conn_fail_signal_(errorCode, Aws::Crt::ErrorDebugString(errorCode));
    } else {
        HJ_INFO("Connection completed with return code %d\n", returnCode);
        isConnect_.store(true);
        conn_signal_();
        conchange_signal_(true);
    }

    connCond_.notify_one();
}

void AwsConnectionManager::connectionSuccessCb(
                          Aws::Crt::Mqtt::MqttConnection &, 
                          Aws::Crt::Mqtt::OnConnectionSuccessData* succdata)
{
    HJ_INFO("MQTT Connection success:%d\n", succdata->returnCode);
    isConnect_.store(true);
}

void AwsConnectionManager::connectionFalureCb(
                          Aws::Crt::Mqtt::MqttConnection &,
                          Aws::Crt::Mqtt::OnConnectionFailureData* faildata)
{
    HJ_ERROR("MQTT Connection fail:%s\n", Aws::Crt::ErrorDebugString(faildata->error));
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
    conchange_signal_(true);
    isConnect_.store(true);
}

void AwsConnectionManager::connectThreadFunc()
{
    bool ret = false;

    std::mutex mtx;
    std::unique_lock<std::mutex> lc(mtx);

    connRun_ = true;

    while (!isConnect_.load() && connRun_) {
        HJ_INFO("MQTT Connection [%s] start...\n", clientId_.c_str());
        ret = connptr_->Connect(clientId_.data(), false, keepAliveTimeSecs_, pingTimeoutMs_, protocolOperationTimeoutMs_);
        if (!ret) { 
            HJ_ERROR("MQTT Connection failed with error %s\n", Aws::Crt::ErrorDebugString(connptr_->LastError()));
            sleep(connFailIntervalSec_);
            continue;
        }
        connCond_.wait(lc);
        if (!isConnect_.load())
            sleep(connFailIntervalSec_);
    }

    connRun_ = false;
    HJ_INFO("MQTT Connection exit\n");
}

}
