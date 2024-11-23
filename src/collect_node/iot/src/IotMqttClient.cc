#include "IotMqttClient.h"
#include "Utils.h"
#include <future>
#include "log.h"
#include <sys/prctl.h>

namespace collect_node_iot {
extern utils::CountDownLatch cdlatch_g;

IotMqttClient::IotMqttClient(const std::string& thingName):
    subRun_(false),
    subIotFlag_(0x00)
{
    downChanTopic_ = "aiper/things/" + thingName + "/downChan";
    upChanTopic_ = "aiper/things/" + thingName + "/upChan";
    downChanCloudTopic_ = "aiper/things/" + thingName + "/cloud/device/downChan";
    upChanCloudTopic_ = "aiper/things/" + thingName + "/device/cloud/upChan";
    reportAppTopic_ = "aiper/things/" + thingName + "/app/report";
    reportShadowTopic_ = "aiper/things/" + thingName + "/shadow/report";
    reportCloudTopic_ = "aiper/things/" + thingName + "/cloud/report";

    utils::DevInfo devInfo("/tmp/devInfo.json");
    bigDataTopic_ = "aiper/things/" + thingName + "/" + devInfo.model() + "/bigdata/report";
    HJ_INFO("big data topic: [%s]\n", bigDataTopic_.c_str());
}

IotMqttClient::~IotMqttClient()
{
    /*
    auto conn = AwsConnectionManager::instance()->getAwsConnection();
    std::promise<void> unsubscribeFinishedPromise;
    conn->Unsubscribe(downChanTopic_.c_str(), [&](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int) {
        unsubscribeFinishedPromise.set_value();
    });
    unsubscribeFinishedPromise.get_future().wait();
    */
}

void IotMqttClient::connectSlot()
{
    HJ_INFO("Iot MQTT connect succ back call!\n");

    if (subRun_.load()) {
        HJ_INFO("sub in run, skip\n");
        return;
    }
    
    if (subIotFlag_.load() == (SUB_IOT_TOPIC_ACK | SUB_IOT_CLOUD_TOPIC_ACK)) {
        HJ_INFO("sub iot topic done, skip");
        return;
    }

    subIotCoreThread_ = std::thread(&IotMqttClient::subToDownChanThread, this);
    subIotCoreThread_.detach();
}

void IotMqttClient::disconnSlot()
{
    subIotFlag_.store(0);
}

void IotMqttClient::subToDownChanThread()
{
    ros::Rate loop_rate(100);
    HJ_INFO("Subscribe to IOT core...\n");
    subRun_.store(true);
    while(subIotFlag_.load() != (SUB_IOT_TOPIC_ACK | SUB_IOT_CLOUD_TOPIC_ACK)) {
        if (!(subIotFlag_.load() & SUB_IOT_TOPIC_ACK)) {
            this->subToDownChan();
        }

        if (!(subIotFlag_.load() & SUB_IOT_CLOUD_TOPIC_ACK)) {
            this->subToCloudDownChan();
        }
        loop_rate.sleep();
    }
    HJ_INFO("Subscribe to IOT core done!\n");
    subRun_.store(false);
    cdlatch_g.countDown();
}

void IotMqttClient::subToDownChan()
{   
    std::promise<bool> subscribeFinishedPromise;
    auto onSubAck =
        [&](Aws::Crt::Mqtt::MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, Aws::Crt::Mqtt::QOS QoS, int errorCode) {
            if (errorCode)
            {
                HJ_ERROR("Subscribe failed with error %s\n", Aws::Crt::ErrorDebugString(errorCode));
                subscribeFinishedPromise.set_value(false);
                return;
            }
            else
            {
                if (!packetId || QoS == AWS_MQTT_QOS_FAILURE)
                {
                    HJ_ERROR("Subscribe rejected by the broker.");
                    subscribeFinishedPromise.set_value(false);
                    return;
                }
                else
                {
                    HJ_INFO("Subscribe on topic %s on packetId %d Succeeded\n", topic.c_str(), packetId);
                }
            }
            subscribeFinishedPromise.set_value(true);
        };
    
    AwsConnectionManager::instance()->getAwsConnection()->Subscribe(
        downChanTopic_.data(), 
        AWS_MQTT_QOS_AT_LEAST_ONCE, 
            Aws::Crt::Mqtt::OnMessageReceivedHandler(boost::bind(&IotMqttClient::onDownChanMessageCb, 
                            this,
                            boost::placeholders::_1,
                            boost::placeholders::_2,
                            boost::placeholders::_3,
                            boost::placeholders::_4,
                            boost::placeholders::_5,
                            boost::placeholders::_6)),
        onSubAck);

    if (subscribeFinishedPromise.get_future().get()) {
        subIotFlag_ |= SUB_IOT_TOPIC_ACK;
        HJ_INFO("Subscribe to [%s] success\n", downChanTopic_.c_str());
    } else {
        HJ_INFO("Subscribe to [%s] fail\n", downChanTopic_.c_str());
    }
}

void IotMqttClient::subToCloudDownChan()
{
    std::promise<bool> subscribeFinishedPromise;
    auto onSubAck =
        [&](Aws::Crt::Mqtt::MqttConnection &, uint16_t packetId, const Aws::Crt::String &topic, Aws::Crt::Mqtt::QOS QoS, int errorCode) {
            if (errorCode)
            {
                HJ_ERROR("Subscribe failed with error %s\n", Aws::Crt::ErrorDebugString(errorCode));
                subscribeFinishedPromise.set_value(false);
                return;
            }
            else
            {
                if (!packetId || QoS == AWS_MQTT_QOS_FAILURE)
                {
                    HJ_ERROR("Subscribe rejected by the broker.");
                    subscribeFinishedPromise.set_value(false);
                    return;
                }
                else
                {
                    HJ_INFO("Subscribe on topic %s on packetId %d Succeeded\n", topic.c_str(), packetId);
                }
            }
            subscribeFinishedPromise.set_value(true);
        };
    
    AwsConnectionManager::instance()->getAwsConnection()->Subscribe(
        downChanCloudTopic_.data(), 
        AWS_MQTT_QOS_AT_LEAST_ONCE, 
            Aws::Crt::Mqtt::OnMessageReceivedHandler(boost::bind(&IotMqttClient::onCloudDownChanMessageCb, 
                            this,
                            boost::placeholders::_1,
                            boost::placeholders::_2,
                            boost::placeholders::_3,
                            boost::placeholders::_4,
                            boost::placeholders::_5,
                            boost::placeholders::_6)),
        onSubAck);

    if (subscribeFinishedPromise.get_future().get()) {
        subIotFlag_ |= SUB_IOT_CLOUD_TOPIC_ACK;
        HJ_INFO("Subscribe to [%s] success\n", downChanCloudTopic_.c_str());
    } else {
        HJ_INFO("Subscribe to [%s] fail\n", downChanCloudTopic_.c_str());
    }
}

void IotMqttClient::onDownChanMessageCb(Aws::Crt::Mqtt::MqttConnection &connection, 
                     const Aws::Crt::String &topic, 
                     const Aws::Crt::ByteBuf &payload, 
                     bool dup, 
                     Aws::Crt::Mqtt::QOS qos, 
                     bool retain)
{
    if (appMqttCb_)
        appMqttCb_(payload, IOT_TOPIC);
}

void IotMqttClient::onCloudDownChanMessageCb(Aws::Crt::Mqtt::MqttConnection &connection, 
                     const Aws::Crt::String &topic, 
                     const Aws::Crt::ByteBuf &payload, 
                     bool dup, 
                     Aws::Crt::Mqtt::QOS qos, 
                     bool retain)
{
    if (appMqttCb_)
        appMqttCb_(payload, IOT_CLOUD_TOPIC);
}

void IotMqttClient::mqttAppReport(const Aws::Crt::String& buffer)
{
    Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromArray((const uint8_t *)buffer.data(), buffer.length());

    auto onPublishComplete = [](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int) {};
    
    auto conn = AwsConnectionManager::instance()->getAwsConnection();
    
    conn->Publish(reportAppTopic_.c_str(), AWS_MQTT_QOS_AT_MOST_ONCE, false, payload, onPublishComplete);
}

void IotMqttClient::mqttResp(const Aws::Crt::String& buffer, uint8_t route)
{
    Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromArray((const uint8_t *)buffer.data(), buffer.length());

    auto onPublishComplete = [](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int code) {};
    
    auto conn = AwsConnectionManager::instance()->getAwsConnection();
    
    if (route == IOT_TOPIC) {
        conn->Publish(upChanTopic_.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPublishComplete);
    } else if (route == IOT_CLOUD_TOPIC) {
        conn->Publish(upChanCloudTopic_.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPublishComplete);
    }
}

void IotMqttClient::mqttCloudReport(const Aws::Crt::String& buffer)
{
    Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromArray((const uint8_t *)buffer.data(), buffer.length());

    auto onPublishComplete = [](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int code) {
        if (code != AWS_OP_SUCCESS) {
            HJ_ERROR("Mqtt publish cloud fail\n");
        }
    };
    
    auto conn = AwsConnectionManager::instance()->getAwsConnection();
    
    conn->Publish(reportCloudTopic_.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPublishComplete);
}

void IotMqttClient::mqttBigDataReport(const Aws::Crt::String& buffer)
{
    Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromArray((const uint8_t *)buffer.data(), buffer.length());

    auto onPublishComplete = [](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int code) {
        if (code != AWS_OP_SUCCESS) {
            HJ_ERROR("Mqtt publish bigdata fail\n");
        }
    };
    
    auto conn = AwsConnectionManager::instance()->getAwsConnection();
    
    conn->Publish(bigDataTopic_.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPublishComplete);
}

void IotMqttClient::updateShadowReport(const Aws::Crt::String& buffer)
{
    Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromArray((const uint8_t *)buffer.data(), buffer.length());

    auto onPublishComplete = [](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int err) {
        HJ_INFO("pub update mqtt shadow\n");
        if (err != AWS_OP_SUCCESS)
        {
            HJ_ERROR("Failed to update shadow report\n");
            return;
        }

        HJ_INFO("Successfully updated shadow report\n");
    };
    
    auto conn = AwsConnectionManager::instance()->getAwsConnection();
    
    conn->Publish(reportShadowTopic_.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload, onPublishComplete);
}


}