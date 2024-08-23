#include "IotShadowClient.h"
#include "log.h"
#include "error.h"
#include <aws/crt/Api.h>
#include <future>
#include <aws/iotshadow/GetShadowRequest.h>
#include <sys/prctl.h>
#include <pthread.h>
namespace collect_node_iot {

IotShadowClient::IotShadowClient(const std::string& thingName)
    :thingName_(thingName),
    shadowClient_(AwsConnectionManager::instance()->getAwsConnection()),
    subIotFail_(0xFF)
{

}

IotShadowClient::~IotShadowClient()
{

}

void IotShadowClient::connectSlot() 
{
    HJ_INFO("Iot SHADWO connect succ back call!\n");
 
    subIotCoreThread_ = std::thread(&IotShadowClient::subscribeToCore, this);
    subIotCoreThread_.detach();
}

void IotShadowClient::disconnSlot()
{
    subIotFail_.store(0xff);
}

void IotShadowClient::subscribeToCore()
{
    HJ_INFO("Subscribe to IOT core...\n");
    prctl(PR_SET_NAME, "subscribeToCore");
    ros::Rate loop_rate(100);

    subToDeleteShadowAccepted();
    subToDeleteShadowRejected();
    subToGetShadowAccepted();
    subToGetShadowRejected();
    subToUpdateShadowAccepted();
    subToUpdateShadowRejected();
    subToShadowDeltaUpdatedEvents();
    subToShadowUpdatedEvents();

    while(subIotFail_.load() != 0x00) {
        if (subIotFail_.load() & SHADOW_DEL_ACCEP_FAIL) {
            subToDeleteShadowAccepted();
        }
        if (subIotFail_.load() & SHADOW_DEL_REJ_FAIL) {
            subToDeleteShadowRejected();
        }
        if (subIotFail_.load() & SHADOW_GET_ACCEP_FAIL) {
            subToGetShadowAccepted();
        }
        if (subIotFail_.load() & SHADOW_GET_REJ_FAIL) {
            subToGetShadowRejected();
        }
        if (subIotFail_.load() & SHADOW_UPDATE_ACCEP_FAIL) {
            subToUpdateShadowAccepted();
        }
        if (subIotFail_.load() & SHADOW_UPDATE_REJ_FAIL) {
            subToUpdateShadowRejected();
        }
        if (subIotFail_.load() & SHADOW_UPDATE_DELTA_FAIL) {
            subToShadowDeltaUpdatedEvents();
        }
        if (subIotFail_.load() & SHADOW_UPDATE_DOC_FAIL) {
            subToShadowUpdatedEvents();
        }

        loop_rate.sleep();
    }
    
    HJ_INFO("Subscribe to IOT core done!\n");
}

/* 
Topic: ShadowTopicPrefix/delete/accepted 
Meaning: The delete request was accepted and AWS IoT deleted the shadow. 
Device: The actions necessary to accommodate the deleted shadow, 
        such as stop publishing updates.
*/
uint8_t IotShadowClient::subToDeleteShadowAccepted()
{   
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to delete shadow document accepted: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_DEL_ACCEP_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::DeleteShadowSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToDeleteShadowAccepted(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::deleteShadowAcceptedCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_DEL_ACCEP_FAIL;
    }

    return 0;
}

/*
Topic: ShadowTopicPrefix/delete/rejected
Meaning: The delete request was rejected by AWS IoT and 
         the shadow was not deleted. The message body contains 
         the error information. 
Device: Respond to the error message in the message body.
*/
uint8_t IotShadowClient::subToDeleteShadowRejected()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to delete shadow document rejected: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_DEL_REJ_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::DeleteShadowSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToDeleteShadowRejected(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::deleteShadowRejectedCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_DEL_REJ_FAIL;  
    }

    return 0;
}

/*
Topic: ShadowTopicPrefix/get/accepted
Meaning: The get request was accepted by AWS IoT, and the message body 
         contains the current shadow document. 
Device: The actions necessary to process the state document in the message body.
*/
uint8_t IotShadowClient::subToGetShadowAccepted()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to get shadow document accepted: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_GET_ACCEP_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::GetShadowSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToGetShadowAccepted(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::getShadowAcceptedCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_GET_ACCEP_FAIL;  
    }

    return 0;
}

/*
Topic: ShadowTopicPrefix/get/rejected
Meaning: The get request was rejected by AWS IoT, and the message body 
         contains the error information. 
Device: Respond to the error message in the message body.
*/
uint8_t IotShadowClient::subToGetShadowRejected()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to get shadow document rejected: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_GET_REJ_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::GetShadowSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToGetShadowRejected(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::getShadowRejectedCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_GET_REJ_FAIL;  
    }

    return 0;
}

/*
Tpoic: ShadowTopicPrefix/update/accepted
Meaning: The update request was accepted by AWS IoT, and the message body 
         contains the current shadow document. 
Device: Confirm the updated data in the message body matches the device state.
*/
uint8_t IotShadowClient::subToUpdateShadowAccepted()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to update shadow accepted: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_UPDATE_ACCEP_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::UpdateShadowSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToUpdateShadowAccepted(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::updateShadowAcceptedCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_UPDATE_ACCEP_FAIL;  
    }

    return 0;
}

/*
Topic: ShadowTopicPrefix/update/rejected
Meaning: The update request was rejected by AWS IoT, and the message body 
         contains the error information. 
Device: Respond to the error message in the message body.
*/
uint8_t IotShadowClient::subToUpdateShadowRejected()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to update shadow rejected: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_UPDATE_REJ_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::UpdateShadowSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToUpdateShadowRejected(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::updateShadowRejectedCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_UPDATE_REJ_FAIL;  
    }

    return 0;
}

/*
Topic: ShadowTopicPrefix/update/delta
Meaning: The shadow document was updated by a request to AWS IoT, and the message 
         body contains the changes requested. 
Device: Update the device's state to match the desired state in the message body.
*/
uint8_t IotShadowClient::subToShadowDeltaUpdatedEvents()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to delta update events: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_UPDATE_DELTA_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::ShadowDeltaUpdatedSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToShadowDeltaUpdatedEvents(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::shadowDeltaUpdatedEventsCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_UPDATE_DELTA_FAIL;  
    }

    return 0;
}

/*
Topic: ShadowTopicPrefix/update/documents
Meaning: An update to the shadow was recently completed, and the message body 
         contains the current shadow document. 
Device: Confirm the updated state in the message body matches the device's state.
*/
uint8_t IotShadowClient::subToShadowUpdatedEvents()
{
    std::promise<bool> subCompletePromise;
    auto subAck = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error subscribing to update events: %s\n", Aws::Crt::ErrorDebugString(ioErr));
            subCompletePromise.set_value(false);
            return;
        }

        subIotFail_.store(subIotFail_.load() & ~SHADOW_UPDATE_DOC_FAIL);
        subCompletePromise.set_value(true);
    }; 

    Aws::Iotshadow::ShadowUpdatedSubscriptionRequest req;
    req.ThingName = Aws::Crt::String(thingName_.c_str());

    shadowClient_.SubscribeToShadowUpdatedEvents(
        req,
        AWS_MQTT_QOS_AT_LEAST_ONCE,
        boost::bind(&IotShadowClient::shadowUpdatedEventsCb, this, 
                    boost::placeholders::_1, boost::placeholders::_2),
        subAck      
    );

    if (!subCompletePromise.get_future().get()) {
        return SHADOW_UPDATE_DOC_FAIL;  
    }

    return 0;
}

void IotShadowClient::deleteShadowAcceptedCb(Aws::Iotshadow::DeleteShadowResponse * resp, int ioErr)
{
    if (!resp)
        return;

    if (delShadowAccCb_)
        delShadowAccCb_(resp, ioErr);
}

void IotShadowClient::deleteShadowRejectedCb(Aws::Iotshadow::ErrorResponse * err, int ioErr)
{
    if (!err)
        return;

    if (delShadowRejCb_)
        delShadowRejCb_(err, ioErr);
}

void IotShadowClient::getShadowAcceptedCb(Aws::Iotshadow::GetShadowResponse * resp, int ioErr)
{
    if (!resp)
        return;

    if (getShadowAccCb_)
        getShadowAccCb_(resp, ioErr);
}

void IotShadowClient::getShadowRejectedCb(Aws::Iotshadow::ErrorResponse * err, int ioErr)
{
    if (!err)
        return;

    if (getShadowRejCb_)
        getShadowRejCb_(err, ioErr);
}

void IotShadowClient::updateShadowAcceptedCb(Aws::Iotshadow::UpdateShadowResponse * resp, int ioErr)
{
    if (!resp)
        return;

    if (updateShadowAccCb_)
        updateShadowAccCb_(resp, ioErr);
}

void IotShadowClient::updateShadowRejectedCb(Aws::Iotshadow::ErrorResponse * err, int ioErr)
{
    if (!err)
        return;

    if (updateShadowRejCb_)
        updateShadowRejCb_(err, ioErr);
}

void IotShadowClient::shadowDeltaUpdatedEventsCb(Aws::Iotshadow::ShadowDeltaUpdatedEvent * events, int ioErr)
{
    if (!events)
        return;

    if (updateDeltaCb_)
        updateDeltaCb_(events, ioErr);
}

void IotShadowClient::shadowUpdatedEventsCb(Aws::Iotshadow::ShadowUpdatedEvent * events, int)
{
    return;
}

//TODO:pub 内不要使用future
bool IotShadowClient::pubGetShadowDoc(const Aws::Crt::String& uuid, int& code)
{
    Aws::Iotshadow::GetShadowRequest shadowGetRequest;
    if (thingName_.empty())
        return false;
    
    shadowGetRequest.ThingName = Aws::Crt::String(thingName_.c_str());
    shadowGetRequest.ClientToken = uuid;
    std::promise<bool> pubCompletePromise;

    auto publishCompleted = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS)
        {
            HJ_ERROR("Failed to pub shadow get, error %s\n", Aws::Crt::ErrorDebugString(ioErr));
            code = ioErr;
            pubCompletePromise.set_value(false);
            return;
        }

        code = IOT_OK;
        pubCompletePromise.set_value(true);
    };

    shadowClient_.PublishGetShadow(shadowGetRequest, AWS_MQTT_QOS_AT_LEAST_ONCE, publishCompleted);
    
    if(pubCompletePromise.get_future().get())
        return true;
    else
        return false;
}

bool IotShadowClient::pubDelShadowDoc(const Aws::Crt::String& uuid, int& code)
{
    if (thingName_.empty())
        return false;

    Aws::Iotshadow::DeleteShadowRequest shadowDelRequest;
    shadowDelRequest.ThingName = Aws::Crt::String(thingName_.c_str());
    shadowDelRequest.ClientToken = uuid;
    std::promise<bool> pubCompletePromise;

    auto publishCompleted = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS)
        {
            HJ_ERROR("Failed to pub shadow del, error %s\n", Aws::Crt::ErrorDebugString(ioErr));
            code = ioErr;
            pubCompletePromise.set_value(false);
            return;
        }

        code = IOT_OK;
        pubCompletePromise.set_value(true);
    };

    shadowClient_.PublishDeleteShadow(shadowDelRequest, AWS_MQTT_QOS_AT_LEAST_ONCE, publishCompleted);

    if(pubCompletePromise.get_future().get())
        return true;
    else
        return false;
}

bool IotShadowClient::pubUpdateShadowDoc(const Aws::Crt::String& uuid, const Aws::Crt::JsonObject& reportJson, uint32_t ver, Aws::Crt::Mqtt::QOS qos, int& code)
{
    if (thingName_.empty())
        return false;
        
    Aws::Iotshadow::ShadowState state;

    state.Reported = reportJson;

    Aws::Iotshadow::UpdateShadowRequest updateShadowRequest;
    updateShadowRequest.ThingName = Aws::Crt::String(thingName_.c_str());


    if (qos == AWS_MQTT_QOS_AT_LEAST_ONCE) {
        updateShadowRequest.ClientToken = uuid;
    }
    
    updateShadowRequest.State = state;
    if (ver != 0)
        updateShadowRequest.Version = ver;

    auto publishCompleted = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS)
        {
            HJ_ERROR("Failed to pub shadow update, error %s\n", Aws::Crt::ErrorDebugString(ioErr));
            //code = ioErr;
            return;
        }

        HJ_INFO("pub shadow update success\n");
        //code = IOT_OK;
    };

    shadowClient_.PublishUpdateShadow(updateShadowRequest, qos, publishCompleted);
    
    return true;
}

bool IotShadowClient::pubClearShadowDesire(const Aws::Crt::JsonObject& json)
{    
    if (thingName_.empty())
        return false;
        
    Aws::Iotshadow::ShadowState state;

    state.Desired = json;

    Aws::Iotshadow::UpdateShadowRequest updateShadowRequest;
    updateShadowRequest.ThingName = Aws::Crt::String(thingName_.c_str());

    updateShadowRequest.State = state;

    auto publishCompleted = [&](int ioErr) {
        if (ioErr != AWS_OP_SUCCESS)
        {
            HJ_ERROR("Failed to pub shadow clear desire, error %s\n", Aws::Crt::ErrorDebugString(ioErr));
            //code = ioErr;
            return;
        }

        HJ_INFO("pub shadow clear desire success\n");
        //code = IOT_OK;
    };

    shadowClient_.PublishUpdateShadow(updateShadowRequest, AWS_MQTT_QOS_AT_LEAST_ONCE, publishCompleted);
    
    return true;
}

}// namespace collect_node_iot