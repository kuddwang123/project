/**
 * @ Author: lou.pan
 * @ Create Time: 2024-04-16 19:16:11
 * @ Modified by: lou.pan
 * @ Modified time: 2024-05-16 09:35:47
 * @ Description: Handle shadow event lifetime and wrap shadow client
 */


#include "node_factory.h"
#include "hj_interface/IotShadow.h"
#include "hj_interface/AppMsg.h"
#include "IotShadowClient.h"
#include <list>
#include <mutex>
#include <condition_variable>
namespace collect_node_iot {
struct getShadowInfo {
    Aws::Crt::Map<Aws::Crt::String, Aws::Crt::JsonView> desire_;
    Aws::Crt::Map<Aws::Crt::String, Aws::Crt::JsonView> meta_;
    //Aws::Crt::String desireStr_;
    //Aws::Crt::String desireMetaStr_;
    Aws::Crt::String msg_;
    Aws::Crt::String uuid_;
    uint64_t ts_;
    int32_t ver_;
    int32_t code_;
    bool isSuccess_;
    bool isTimeOut_;

    std::mutex mutex_;
    std::condition_variable cond_;

    getShadowInfo(Aws::Crt::String uuid):
        uuid_(uuid),
        ts_(0),
        ver_(0),
        code_(0),
        isSuccess_(false),
        isTimeOut_(false) {}
};

struct delShadowInfo {
    Aws::Crt::String msg_;
    Aws::Crt::String uuid_;
    uint64_t ts_;
    int32_t ver_;
    int32_t code_;
    bool isSuccess_;
    bool isTimeOut_;

    std::mutex mutex_;
    std::condition_variable cond_;

    delShadowInfo(Aws::Crt::String uuid):
        uuid_(uuid),
        ts_(0),
        ver_(0),
        code_(0),
        isSuccess_(false),
        isTimeOut_(false) {}
};

struct updateShadowInfo {
    Aws::Crt::String msg_;
    Aws::Crt::String uuid_;
    uint64_t ts_;
    int32_t ver_;
    int32_t code_;
    bool isSuccess_;
    bool isTimeOut_;

    std::mutex mutex_;
    std::condition_variable cond_;

    updateShadowInfo(Aws::Crt::String uuid):
        uuid_(uuid),
        ts_(0),
        ver_(0),
        code_(0),
        isSuccess_(false),
        isTimeOut_(false) {}
};

template<typename T>
class ServiceIotList {
public:
    ServiceIotList(std::string);
    ~ServiceIotList();
    ServiceIotList& operator=(const ServiceIotList&) = delete;

    bool addToList(Aws::Crt::String);
    bool addToList(const std::shared_ptr<T>&);
    std::shared_ptr<T> getSrv(Aws::Crt::String);
    bool removeFromList(Aws::Crt::String);
    bool removeFromList(const std::shared_ptr<T>&);

private:
    std::string tmrName_;
    std::list<std::shared_ptr<T>> list_;
    hj_bf::HJTimer cleanTimeOutElemTimer_;
    std::mutex mutex_;

private:
    void cleanTimeOutElemTimerCb(const hj_bf::HJTimerEvent&);
};

class IotShadowClientLink {
typedef boost::function<void(hj_interface::AppMsg&)> deltaEventInfoDvcFunc;
typedef boost::function<void(const std::vector<hj_interface::AppData>&)> shadowUpdateAccptCb;

public:
    IotShadowClientLink();
    ~IotShadowClientLink();

  //初始化，向aws shadow client注册事件回调
    bool initialize(const std::shared_ptr<IotShadowClient>&, const deltaEventInfoDvcFunc&);
  
  //注册影子更新成功回调
    void setShadowUpdateAcceptCb(const shadowUpdateAccptCb& cb) {shadowUpdateAccptCb_ = cb;}

  //设备更新状态至shadow doc
    bool dvcUpdateShadow(const hj_interface::AppMsg&);
  
  //获取shadow doc
    bool linkGetShadow(const hj_interface::IotShadowRequest&,
                   hj_interface::IotShadowResponse&);
  
  //删除shadow doc
    bool linkDeleteShadow(const hj_interface::IotShadowRequest&,
                      hj_interface::IotShadowResponse&);
  
  //更新shadow doc
    bool linkIotUpdateShadow(const hj_interface::IotShadowRequest&,
                    hj_interface::IotShadowResponse&);

private:
    std::shared_ptr<IotShadowClient> shadowClientPtr_;
    
    std::shared_ptr<ServiceIotList<getShadowInfo>> getShadowServiceLinkList_;
    std::shared_ptr<ServiceIotList<delShadowInfo>> delShadowServiceLinkList_;
    std::shared_ptr<ServiceIotList<updateShadowInfo>> iotCallServiceLinkList_;
    
    deltaEventInfoDvcFunc deltaUpdateCb_;
    shadowUpdateAccptCb shadowUpdateAccptCb_;

private:
    void registCallBackToShadowClient();
    
    void getShadowAcceptCb(Aws::Iotshadow::GetShadowResponse*, int);
    
    void getShadowRejectCb(Aws::Iotshadow::ErrorResponse*, int);

    void delShadowAcceptCb(Aws::Iotshadow::DeleteShadowResponse*, int);
    
    void delShadowRejectCb(Aws::Iotshadow::ErrorResponse*, int);

    void updateShadowAcceptCb(Aws::Iotshadow::UpdateShadowResponse *, int);

    void updateShadowRejectedCb(Aws::Iotshadow::ErrorResponse *, int);

    void shadowUpdateDeltaCb(Aws::Iotshadow::ShadowDeltaUpdatedEvent *, int);
};

}