/**
 * @ Author: lou.pan
 * @ Create Time: 2024-03-27 17:58:56
 * @ Modified by: lou.pan
 * @ Modified time: 2024-05-10 13:47:58
 * @ Description:Aws shadow client SDK package
 */

#include <aws/iotshadow/ErrorResponse.h>
#include <aws/iotshadow/IotShadowClient.h>
#include <aws/iotshadow/ShadowUpdatedEvent.h>
#include <aws/iotshadow/ShadowDeltaUpdatedEvent.h>
#include <aws/iotshadow/ShadowDeltaUpdatedSubscriptionRequest.h>
#include <aws/iotshadow/UpdateShadowRequest.h>
#include <aws/iotshadow/UpdateShadowResponse.h>
#include <aws/iotshadow/UpdateShadowSubscriptionRequest.h>
#include <aws/iotshadow/DeleteShadowSubscriptionRequest.h>
#include <aws/iotshadow/DeleteShadowResponse.h>
#include <aws/iotshadow/GetShadowRequest.h>
#include <aws/iotshadow/GetShadowResponse.h>
#include <aws/iotshadow/DeleteShadowRequest.h>
#include <aws/iotshadow/ShadowUpdatedSubscriptionRequest.h>
#include <aws/iotshadow/GetShadowSubscriptionRequest.h>
#include <aws/iotshadow/UpdateShadowSubscriptionRequest.h>
#include <aws/crt/Types.h>

#include <boost/function.hpp>
#include <string>
#include <thread>
#include <atomic>

#include "AwsConnectionManager.h"
namespace collect_node_iot {
typedef boost::function<void(Aws::Iotshadow::GetShadowResponse*, int)> getShadowAccCb;
typedef boost::function<void(Aws::Iotshadow::DeleteShadowResponse*, int)> delShadowAccCb;
typedef boost::function<void(Aws::Iotshadow::UpdateShadowResponse*, int)> updateShadowAccCb;
typedef boost::function<void(Aws::Iotshadow::ShadowDeltaUpdatedEvent*, int)> updateDeltaCb;
typedef boost::function<void(Aws::Iotshadow::ErrorResponse*, int)> shadowOperRejCb;

//TODO:拆分订阅器和发布器
class IotShadowClient {
public:
    IotShadowClient(const std::string& thing);
    ~IotShadowClient();

  //Aws core连接成功槽函数
    void connectSlot();
  
  //Aws core断连成功槽函数
    void disconnSlot();

  //获取shadow doc话题发布
    bool pubGetShadowDoc(const Aws::Crt::String&, int&);
  
  //清空desire数据
    bool pubClearShadowDesire(const Aws::Crt::JsonObject& json);

  //注册获取shadow doc成功回调
    void setGetShadowAccCb(const getShadowAccCb& cb) { getShadowAccCb_ = cb; }
    
  //注册获取shadow doc失败回调
    void setGetShadowRejCb(const shadowOperRejCb& cb) { getShadowRejCb_ = cb; }

  //删除shadow doc话题发布
    bool pubDelShadowDoc(const Aws::Crt::String&, int&);

  //注册删除shadow doc成功回调
    void setDelShadowAccCb(const delShadowAccCb& cb) { delShadowAccCb_ = cb; }

  //注册删除shadow doc失败回调
    void setDelShadowRejCb(const shadowOperRejCb& cb) { delShadowRejCb_ = cb; }

  //更新shadow doc话题发布
    bool pubUpdateShadowDoc(const Aws::Crt::String&, const Aws::Crt::JsonObject&, uint32_t,  Aws::Crt::Mqtt::QOS, int&);
  
  //注册更新shadow doc成功回调
    void setUpdateShadowAccCb(const updateShadowAccCb& cb) { updateShadowAccCb_ = cb; }
  
  //注册更新shadow doc失败回调
    void setUpdateShadowRejCb(const shadowOperRejCb& cb) { updateShadowRejCb_ = cb; }
 
  //注册shadow delta事件回调
    void setUpdateDeltaCb(const updateDeltaCb& cb) { updateDeltaCb_ = cb; }

private:
    enum SubShadowIotFail{
        SHADOW_DEL_ACCEP_FAIL = 1 << 0,
        SHADOW_DEL_REJ_FAIL = 1 << 1,
        SHADOW_GET_ACCEP_FAIL = 1 << 2,
        SHADOW_GET_REJ_FAIL = 1 << 3,
        SHADOW_UPDATE_ACCEP_FAIL = 1 << 4,
        SHADOW_UPDATE_REJ_FAIL = 1 << 5,
        SHADOW_UPDATE_DELTA_FAIL = 1 << 6,
        SHADOW_UPDATE_DOC_FAIL = 1 << 7
    };
    
    std::string thingName_;
    Aws::Iotshadow::IotShadowClient shadowClient_;
    std::atomic<uint8_t> subIotFail_;
    std::thread subIotCoreThread_;

    getShadowAccCb getShadowAccCb_;
    shadowOperRejCb getShadowRejCb_;

    delShadowAccCb delShadowAccCb_;
    shadowOperRejCb delShadowRejCb_;

    updateShadowAccCb updateShadowAccCb_;
    shadowOperRejCb updateShadowRejCb_;

    updateDeltaCb updateDeltaCb_;

    std::atomic<uint8_t> subRun_;;

private:
    void subscribeToCore();

    uint8_t subToDeleteShadowAccepted();
    uint8_t subToDeleteShadowRejected();
    uint8_t subToGetShadowAccepted();
    uint8_t subToGetShadowRejected();
    uint8_t subToUpdateShadowAccepted();
    uint8_t subToUpdateShadowRejected();
    uint8_t subToShadowDeltaUpdatedEvents();
    uint8_t subToShadowUpdatedEvents();

    void deleteShadowAcceptedCb(Aws::Iotshadow::DeleteShadowResponse *, int);
    void deleteShadowRejectedCb(Aws::Iotshadow::ErrorResponse *, int);
    void getShadowAcceptedCb(Aws::Iotshadow::GetShadowResponse *, int);
    void getShadowRejectedCb(Aws::Iotshadow::ErrorResponse *, int);
    void updateShadowAcceptedCb(Aws::Iotshadow::UpdateShadowResponse *, int);
    void updateShadowRejectedCb(Aws::Iotshadow::ErrorResponse *, int);
    void shadowDeltaUpdatedEventsCb(Aws::Iotshadow::ShadowDeltaUpdatedEvent *, int);
    void shadowUpdatedEventsCb(Aws::Iotshadow::ShadowUpdatedEvent *, int);
};

}