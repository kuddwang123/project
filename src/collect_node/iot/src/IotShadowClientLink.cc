#include "IotShadowClientLink.h"
#include "log.h"
#include "error.h"
#include <algorithm>
#include <aws/crt/UUID.h>
namespace collect_node_iot {

template<typename T>
ServiceIotList<T>::ServiceIotList(std::string TmrName):
    tmrName_(TmrName)
{
    cleanTimeOutElemTimer_ = 
        hj_bf::HJCreateTimer(tmrName_, 30 * 1000 * 1000, &ServiceIotList::cleanTimeOutElemTimerCb, this);
}

template<typename T>
ServiceIotList<T>::~ServiceIotList()
{
   
}

template<typename T>
bool ServiceIotList<T>::addToList(Aws::Crt::String id)
{
    std::lock_guard<std::mutex> lc(mutex_);
    auto it = std::find_if(list_.begin(), list_.end(), [&](const std::shared_ptr<T>& other) {
        return other->uuid == id;
    });

    if (it != list_.end()) {
        return false;
    }
    else {
        auto elem = std::make_shared<T>(id);
        list_.push_back(elem);
        return true;
    }
}

template<typename T>
bool ServiceIotList<T>::addToList(const std::shared_ptr<T>& data)
{
    std::lock_guard<std::mutex> lc(mutex_);
    auto it = std::find_if(list_.begin(), list_.end(), [&](const std::shared_ptr<T>& other) {
        return other->uuid_ == data->uuid_;
    });

    if (it != list_.end()) {
        return false;
    }
    else {
        list_.push_back(data);
        return true;
    }
}

template<typename T>
bool ServiceIotList<T>::removeFromList(Aws::Crt::String id)
{
    std::lock_guard<std::mutex> lc(mutex_);
    auto it = std::find_if(list_.begin(), list_.end(), [&](const std::shared_ptr<T>& other) {
        return other->uuid_ == id;
    });

    if (it != list_.end()) {
        list_.erase(it);
        return true;
    }
    else {
        return false;
    }
}

template<typename T>
bool ServiceIotList<T>::removeFromList(const std::shared_ptr<T>& data)
{
    std::lock_guard<std::mutex> lc(mutex_);
    auto it = std::find_if(list_.begin(), list_.end(), [&](const std::shared_ptr<T>& other) {
        return other->uuid_ == data->uuid_;
    });

    if (it != list_.end()) {
        list_.erase(it);
        return true;
    }
    else {
        return false;
    }
}

template<typename T>
std::shared_ptr<T> ServiceIotList<T>::getSrv(Aws::Crt::String uuid)
{
    std::lock_guard<std::mutex> lc(mutex_);
    auto it = std::find_if(list_.begin(), list_.end(), [&](const std::shared_ptr<T>& other) {
        return other->uuid_ == uuid;
    });

    if (it == list_.end())
        return nullptr;
    
    return *it;
}

template<typename T>
void ServiceIotList<T>::cleanTimeOutElemTimerCb(const hj_bf::HJTimerEvent&)
{
    std::lock_guard<std::mutex> lc(mutex_);
    for (auto it = list_.begin(); it != list_.end();) {
        if (it->get()->isTimeOut_)
            it = list_.erase(it);
        else
            ++it;
    }
}

IotShadowClientLink::IotShadowClientLink():
    getShadowServiceLinkList_(std::make_shared<ServiceIotList<getShadowInfo>>("getShadow")),
    delShadowServiceLinkList_(std::make_shared<ServiceIotList<delShadowInfo>>("delShadow")),
    iotCallServiceLinkList_(std::make_shared<ServiceIotList<updateShadowInfo>>("iotCall"))
{

}

IotShadowClientLink::~IotShadowClientLink()
{
    HJ_INFO("IotShadowClientLink destruct call\n");
}

bool IotShadowClientLink::initialize(
        const std::shared_ptr<IotShadowClient>& shadowClientPtr,
        const deltaEventInfoDvcFunc& cb)
{
    shadowClientPtr_ = shadowClientPtr;
    deltaUpdateCb_ = cb;

    registCallBackToShadowClient();

    return true;
}

void IotShadowClientLink::registCallBackToShadowClient()
{
    shadowClientPtr_->setGetShadowAccCb(boost::bind(&IotShadowClientLink::getShadowAcceptCb, this,
                                        boost::placeholders::_1,
                                        boost::placeholders::_2));
    
    shadowClientPtr_->setGetShadowRejCb(boost::bind(&IotShadowClientLink::getShadowRejectCb, this,
                                        boost::placeholders::_1,
                                        boost::placeholders::_2));
    
    shadowClientPtr_->setDelShadowAccCb(boost::bind(&IotShadowClientLink::delShadowAcceptCb, this,
                                        boost::placeholders::_1,
                                        boost::placeholders::_2));
    
    shadowClientPtr_->setDelShadowRejCb(boost::bind(&IotShadowClientLink::delShadowRejectCb, this,
                                        boost::placeholders::_1,
                                        boost::placeholders::_2));
    
    shadowClientPtr_->setUpdateShadowAccCb(boost::bind(&IotShadowClientLink::updateShadowAcceptCb, this,
                                           boost::placeholders::_1,
                                           boost::placeholders::_2));
    
    shadowClientPtr_->setUpdateShadowRejCb(boost::bind(&IotShadowClientLink::updateShadowRejectedCb, this,
                                           boost::placeholders::_1,
                                           boost::placeholders::_2));
    
    shadowClientPtr_->setUpdateDeltaCb(boost::bind(&IotShadowClientLink::shadowUpdateDeltaCb, this,
                                       boost::placeholders::_1,
                                       boost::placeholders::_2));
}

bool IotShadowClientLink::dvcUpdateShadow(const hj_interface::AppMsg& msg)
{
    Aws::Crt::JsonObject reportData;
    Aws::Crt::JsonObject jsonValueNull;
    jsonValueNull.AsNull();

    if (!shadowClientPtr_) {
        HJ_INFO("shadow client not construct!\n");
        return false;
    }

    if (msg.appdata.empty()) {
        HJ_ERROR("update empty\n");
        return false;
    }

    for (const auto& data:msg.appdata) {
        if (data.payload.empty()) {
            reportData.WithObject(data.key.data(), jsonValueNull);
        } else {
            Aws::Crt::JsonObject subdata(data.payload.c_str());
            if (!subdata.WasParseSuccessful()) {
                HJ_ERROR("payload data invalid:%s\n", data.payload.c_str());
                return false;
            }
            if (subdata.View().IsObject() || subdata.View().IsListType()) {
                reportData.WithObject(data.key.data(), subdata);
            } else {
                HJ_ERROR("payload not object:%s\n", data.payload.c_str());
                continue;
            }
        }
    }

    int ret = 0;
    Aws::Crt::UUID uuid;
    return shadowClientPtr_->pubUpdateShadowDoc(uuid.ToString(), reportData, 0, AWS_MQTT_QOS_AT_MOST_ONCE, ret);
}

void IotShadowClientLink::getShadowRejectCb(Aws::Iotshadow::ErrorResponse* awsRes, int ioErr)
{
    if (!awsRes->ClientToken) {
        return;
    }

    std::shared_ptr<getShadowInfo> srvptr;
    if (!(srvptr = getShadowServiceLinkList_->getSrv(awsRes->ClientToken.value()))) {
        HJ_ERROR("invalid rej res:%s\n", awsRes->ClientToken.value().c_str());
        return;
    }
    
    {
        std::lock_guard<std::mutex> lc(srvptr->mutex_);

        if (srvptr->isTimeOut_) {
            HJ_ERROR("get shadow rej res:%s timeout, do nothing\n", awsRes->ClientToken.value().c_str());
            return;
        }

        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error on getting shadow document reject: %s.\n", getMessageByCode(ioErr).c_str());
            srvptr->code_ = ioErr;
            srvptr->msg_ = Aws::Crt::String(getMessageByCode(ioErr).c_str());
        } else {
            srvptr->code_ = awsRes->Code.value();
            srvptr->msg_ = awsRes->Message.value();
        }

        srvptr->isSuccess_ = false;
    }

    srvptr->cond_.notify_one();
}


void IotShadowClientLink::getShadowAcceptCb(Aws::Iotshadow::GetShadowResponse* awsRes, int ioErr)
{
    if (!awsRes->ClientToken) {
        return;
    }

    std::shared_ptr<getShadowInfo> srvptr;
    if (!(srvptr = getShadowServiceLinkList_->getSrv(awsRes->ClientToken.value()))) {
        //HJ_ERROR("invalid acc res:%s\n", awsRes->ClientToken.value().c_str());
        return;
    }
    
    {
        std::lock_guard<std::mutex> lc(srvptr->mutex_);
        
        if (srvptr->isTimeOut_) {
            HJ_ERROR("get shadow acc res:%s timeout, do nothing\n", awsRes->ClientToken.value().c_str());
            return;
        }

        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error on getting shadow document accept: %s.\n", getMessageByCode(ioErr).c_str());
            srvptr->code_ = ioErr;
            srvptr->msg_ = Aws::Crt::String(getMessageByCode(ioErr).c_str());
            srvptr->isSuccess_ = false;
        } else {
            if (!awsRes->State->Desired || !awsRes->Metadata->Desired) {
                srvptr->code_ = IOT_SHADOW_NO_DESIRE;
                srvptr->msg_ = Aws::Crt::String(getMessageByCode(IOT_SHADOW_NO_DESIRE).c_str());
                srvptr->isSuccess_ = false;
            }
            else {
                //srvptr->desireStr_ = awsRes->State->Desired->View().WriteCompact();
                //srvptr->desireMetaStr_ = awsRes->Metadata->Desired->View().WriteCompact();
                srvptr->desire_ = awsRes->State->Desired->View().GetAllObjects();
                srvptr->meta_ = awsRes->Metadata->Desired->View().GetAllObjects();
                srvptr->ts_ = awsRes->Timestamp->Millis();
                srvptr->ver_ = awsRes->Version.value();
                srvptr->isSuccess_ = true;
            }
        }
    }

    srvptr->cond_.notify_one();
}


bool IotShadowClientLink::linkGetShadow(const hj_interface::IotShadowRequest& req,
                 hj_interface::IotShadowResponse& res)
{
    if (!AwsConnectionManager::instance()->isConnected()) {
        res.iotret.code = IOT_CONNECTION_LOST;
        res.iotret.msg = getMessageByCode(IOT_CONNECTION_LOST);
        return true;
    }

    int ret = 0;
    Aws::Crt::UUID uuid;
    auto srv = std::make_shared<getShadowInfo>(uuid.ToString());
    getShadowServiceLinkList_->addToList(srv);

    if (!shadowClientPtr_->pubGetShadowDoc(uuid.ToString(), ret)) {
        res.iotret.code = ret;
        res.iotret.msg = getMessageByCode(ret);
        return true;
    }

    //TODO:timeout可配置
    {
        std::unique_lock<std::mutex> lc(srv->mutex_);
        if (std::cv_status::timeout == 
                 srv->cond_.wait_for(lc, std::chrono::seconds(2))) {
            srv->isTimeOut_ = true;
            res.iotret.code = IOT_REQUEST_TIMEOUT;
            res.iotret.msg = getMessageByCode(IOT_REQUEST_TIMEOUT);
        } else {
            if (!srv->isSuccess_) {
                res.iotret.code = srv->code_;
                res.iotret.msg = std::string(srv->msg_.data());
            } else {
                hj_interface::AppData appdata;
                for (auto it = srv->desire_.begin(); it != srv->desire_.end(); ++it) {
                    appdata.key = std::string(it->first.data());
                    appdata.payload = std::string(it->second.WriteCompact().data());
                    res.desired.emplace_back(appdata);
                }
                for (auto it = srv->meta_.begin(); it != srv->meta_.end(); ++it) {
                    appdata.key = std::string(it->first.data());
                    appdata.payload = std::string(it->second.WriteCompact().data());
                    res.metadata.emplace_back(appdata);
                }
                //res.desired = std::string(srv->desireStr_.data());
                //res.metadata = std::string(srv->desireMetaStr_.data());
                res.timestamp = srv->ts_;
                res.iotret.code = IOT_OK;
                res.iotret.msg = getMessageByCode(res.iotret.code);
            }
        }
    }

    return true;
}

void IotShadowClientLink::delShadowAcceptCb(Aws::Iotshadow::DeleteShadowResponse* awsRes, int ioErr)
{
    if (!awsRes->ClientToken) {
        return;
    }

    std::shared_ptr<delShadowInfo> srvptr;
    if (!(srvptr = delShadowServiceLinkList_->getSrv(awsRes->ClientToken.value()))) {
        //HJ_ERROR("invalid acc res:%s\n", awsRes->ClientToken.value().c_str());
        return;
    }
    
    {
        std::lock_guard<std::mutex> lc(srvptr->mutex_);
        
        if (srvptr->isTimeOut_) {
            HJ_ERROR("del shadow acc res:%s timeout, do nothing\n", awsRes->ClientToken.value().c_str());
            return;
        }

        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error on deleting shadow document accept: %s.\n", getMessageByCode(ioErr).c_str());
            srvptr->code_ = ioErr;
            srvptr->msg_ = Aws::Crt::String(getMessageByCode(ioErr).c_str());
            srvptr->isSuccess_ = false;
        } else {
            srvptr->ver_ = awsRes->Version.value();
            srvptr->ts_ = awsRes->Timestamp->Millis();
            srvptr->isSuccess_ = true;
        }
    }

    srvptr->cond_.notify_one();
}
    
void IotShadowClientLink::delShadowRejectCb(Aws::Iotshadow::ErrorResponse* awsRes, int ioErr)
{
    if (!awsRes->ClientToken) {
        return;
    }

    std::shared_ptr<delShadowInfo> srvptr;
    if (!(srvptr = delShadowServiceLinkList_->getSrv(awsRes->ClientToken.value()))) {
        HJ_ERROR("invalid rej res:%s\n", awsRes->ClientToken.value().c_str());
        return;
    }
    
    {
        std::lock_guard<std::mutex> lc(srvptr->mutex_);

        if (srvptr->isTimeOut_) {
            HJ_ERROR("del shadow rej res:%s timeout, do nothing\n", awsRes->ClientToken.value().c_str());
            return;
        }

        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error on deleting shadow document reject: %s.\n", getMessageByCode(ioErr).c_str());
            srvptr->code_ = ioErr;
            srvptr->msg_ = Aws::Crt::String(getMessageByCode(ioErr).c_str());
        } else {
            srvptr->code_ = awsRes->Code.value();
            srvptr->msg_ = awsRes->Message.value();
        }

        srvptr->isSuccess_ = false;
    }

    srvptr->cond_.notify_one();
}

bool IotShadowClientLink::linkDeleteShadow(const hj_interface::IotShadowRequest& req,
                      hj_interface::IotShadowResponse& res)
{
    if (!AwsConnectionManager::instance()->isConnected()) {
        res.iotret.code = IOT_CONNECTION_LOST;
        res.iotret.msg = getMessageByCode(IOT_CONNECTION_LOST);
        return true;
    }

    int ret = 0;
    Aws::Crt::UUID uuid;
    auto srv = std::make_shared<delShadowInfo>(uuid.ToString());
    delShadowServiceLinkList_->addToList(srv);

    if (!shadowClientPtr_->pubDelShadowDoc(uuid.ToString(), ret)) {
        res.iotret.code = ret;
        res.iotret.msg = getMessageByCode(ret);
        return true;
    }

    //TODO:timeout可配置
    {
        std::unique_lock<std::mutex> lc(srv->mutex_);
        if (std::cv_status::timeout == 
                 srv->cond_.wait_for(lc, std::chrono::seconds(2))) {
            srv->isTimeOut_ = true;
            res.iotret.code = IOT_REQUEST_TIMEOUT;
            res.iotret.msg = getMessageByCode(IOT_REQUEST_TIMEOUT);
        } else {
            if (!srv->isSuccess_) {
                res.timestamp = srv->ts_;
                res.iotret.code = srv->code_;
                res.iotret.msg = std::string(srv->msg_.data());
            } else {
                res.timestamp = srv->ts_;
                res.iotret.code = IOT_OK;
                res.iotret.msg = getMessageByCode(res.iotret.code);              
            }
        }
    }

    return true;
}

void IotShadowClientLink::updateShadowAcceptCb(Aws::Iotshadow::UpdateShadowResponse * awsRes, int ioErr)
{
    if (shadowUpdateAccptCb_) {
        std::vector<hj_interface::AppData> appdataV;
        auto state = awsRes->State->Reported;
        if (state) {
            auto updatemap = state->View().GetAllObjects();
            for (auto it = updatemap.begin(); it != updatemap.end(); ++it) {
                hj_interface::AppData appdata;
                appdata.key = std::string(it->first.data());
                appdata.payload = std::string(it->second.WriteCompact().data());
                appdataV.emplace_back(appdata);
            }
            shadowUpdateAccptCb_(appdataV);
        }
    }

    if (!awsRes->ClientToken) {
        return;
    }

    std::shared_ptr<updateShadowInfo> srvptr;
    if (!(srvptr = iotCallServiceLinkList_->getSrv(awsRes->ClientToken.value()))) {
        //HJ_ERROR("invalid acc res:%s\n", awsRes->ClientToken.value().c_str());
        return;
    }
    
    {
        std::lock_guard<std::mutex> lc(srvptr->mutex_);
        
        if (srvptr->isTimeOut_) {
            HJ_ERROR("update shadow acc res:%s timeout, do nothing\n", awsRes->ClientToken.value().c_str());
            return;
        }

        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error on updateing shadow document accept: %s.\n", getMessageByCode(ioErr).c_str());
            srvptr->code_ = ioErr;
            srvptr->msg_ = Aws::Crt::String(getMessageByCode(ioErr).c_str());
            srvptr->isSuccess_ = false;
        } else {
            srvptr->ver_ = awsRes->Version.value();
            srvptr->ts_ = awsRes->Timestamp->Millis();
            srvptr->isSuccess_ = true;
        }
    }

    srvptr->cond_.notify_one();
}

void IotShadowClientLink::updateShadowRejectedCb(Aws::Iotshadow::ErrorResponse * awsRes, int ioErr)
{
    if (!awsRes->ClientToken) {
        return;
    }

    std::shared_ptr<updateShadowInfo> srvptr;
    if (!(srvptr = iotCallServiceLinkList_->getSrv(awsRes->ClientToken.value()))) {
        HJ_ERROR("invalid rej res:%s\n", awsRes->ClientToken.value().c_str());
        return;
    }
    
    {
        std::lock_guard<std::mutex> lc(srvptr->mutex_);

        if (srvptr->isTimeOut_) {
            HJ_ERROR("update shadow rej res:%s timeout, do nothing\n", awsRes->ClientToken.value().c_str());
            return;
        }

        if (ioErr != AWS_OP_SUCCESS) {
            HJ_ERROR("Error on updating shadow document reject: %s.\n", getMessageByCode(ioErr).c_str());
            srvptr->code_ = ioErr;
            srvptr->msg_ = Aws::Crt::String(getMessageByCode(ioErr).c_str());
        } else {
            srvptr->code_ = awsRes->Code.value();
            srvptr->msg_ = awsRes->Message.value();
        }

        srvptr->isSuccess_ = false;
    }

    srvptr->cond_.notify_one();
}


bool IotShadowClientLink::linkIotUpdateShadow(const hj_interface::IotShadowRequest& req,
                    hj_interface::IotShadowResponse& res)
{   
    Aws::Crt::JsonObject reportData;

    if (req.appdata.empty()) {
        HJ_ERROR("update empty\n");
        return false;
    }

    if (!AwsConnectionManager::instance()->isConnected()) {
        res.iotret.code = IOT_CONNECTION_LOST;
        res.iotret.msg = getMessageByCode(IOT_CONNECTION_LOST);
        return true;
    }

    for (const auto& data:req.appdata) {
        Aws::Crt::JsonObject subdata(data.payload.data());
        if (!subdata.WasParseSuccessful()) {
            HJ_ERROR("app data invalid:%s\n", data.payload.c_str());
            res.iotret.code = IOT_JSON_INVALID;
            res.iotret.msg = getMessageByCode(IOT_JSON_INVALID);
            return true;
        }
        reportData.WithObject(data.key.data(), subdata);
    }

    //Aws::Crt::JsonObject reportData(Aws::Crt::String(req.appdata.c_str()));

    #if 0
    if (req.appdata.empty() || !reportData.WasParseSuccessful()) {
        HJ_ERROR("app data invalid:%s\n", req.appdata.c_str());
        res.iotret.code = IOT_JSON_INVALID;
        res.iotret.msg = getMessageByCode(IOT_JSON_INVALID);
        return true;
    }
    #endif

    int ret = 0;
    Aws::Crt::UUID uuid;
    auto srv = std::make_shared<updateShadowInfo>(uuid.ToString());
    iotCallServiceLinkList_->addToList(srv);

    if (!shadowClientPtr_->pubUpdateShadowDoc(uuid.ToString(), reportData, 0, AWS_MQTT_QOS_AT_LEAST_ONCE, ret)) {
        res.iotret.code = ret;
        res.iotret.msg = getMessageByCode(ret);
        return true;
    }

    //TODO:timeout可配置
    {
        std::unique_lock<std::mutex> lc(srv->mutex_);
        if (std::cv_status::timeout == 
                 srv->cond_.wait_for(lc, std::chrono::seconds(2))) {
            srv->isTimeOut_ = true;
            res.iotret.code = IOT_REQUEST_TIMEOUT;
            res.iotret.msg = getMessageByCode(IOT_REQUEST_TIMEOUT);
        } else {
            if (!srv->isSuccess_) {
                res.timestamp = srv->ts_;
                res.iotret.code = srv->code_;
                res.iotret.msg = std::string(srv->msg_.data());
            } else {
                res.timestamp = srv->ts_;
                res.iotret.code = IOT_OK;
                res.iotret.msg = getMessageByCode(res.iotret.code);
            }
        }
    }

    return true;
}

void IotShadowClientLink::shadowUpdateDeltaCb(Aws::Iotshadow::ShadowDeltaUpdatedEvent *events, int ioErr)
{
    if (!events->State || !events->Metadata)
        return;

    if (!deltaUpdateCb_)
        return;

    auto desire = events->State;
    auto meta = events->Metadata;

    Aws::Crt::JsonObject desireEmpty;
    Aws::Crt::JsonObject jsonValueNull;
    jsonValueNull.AsNull();

    hj_interface::AppMsg iot;
    hj_interface::AppData appdata;
    
    auto desiremap = desire->View().GetAllObjects();
    for (auto it = desiremap.begin(); it != desiremap.end(); ++it) {
        appdata.key = std::string(it->first.data());
        appdata.payload = std::string(it->second.WriteCompact().data());
        iot.appdata.emplace_back(appdata);
        desireEmpty.WithObject(it->first.data(), jsonValueNull);
    }

    auto metamap = meta->View().GetAllObjects();
    for (auto it = metamap.begin(); it != metamap.end(); ++it) {
        appdata.key = std::string(it->first.data());
        appdata.payload = std::string(it->second.WriteCompact().data());
        iot.metadata.emplace_back(appdata);
    }

    shadowClientPtr_->pubClearShadowDesire(desireEmpty);
    //iot.appdata = std::string(desire->View().WriteCompact().c_str());
    //iot.metadata = std::string(meta->View().WriteCompact().c_str());
    iot.timestamp = events->Timestamp->Millis();

    deltaUpdateCb_(iot);
}

}