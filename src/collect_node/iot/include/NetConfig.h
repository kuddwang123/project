/**
 *@Author: lou.pan
 *@Create Time: 2024-6-20 11:00:00
 *@Modified by: lou.pan
 *@Modified time: 2024-6-20 11:11:29
 *@Description: Net configure class
 */

#include "AppDataRouter.h"
#include "hj_interface/AppMsg.h"
#include <thread>
#include "Utils.h"
#include "BuryPoint.h"
namespace collect_node_iot {
class NetConfig {
public:
    NetConfig(const std::shared_ptr<AppDataRouter>& appRouterPtr,
        std::string certfile, const NetCfgBuryPointPtr& netbp);
    ~NetConfig() {};

    bool netConfig(const rapidjson::Document& config, uint8_t from);
    bool wifiSet(const std::string& ssid, const std::string& pwd,
        const std::string& iv, uint8_t from);

private:
    enum configState{
        kNONE,
        kNETCONFIG,
        kWIFICONFIG
    };

    std::shared_ptr<AppDataRouter> appRouterPtr_;
    configState state_;
    std::string certfile_;
    NetCfgBuryPointPtr buryPointer_;
    std::string devInfoJsonStr_;
    utils::DevInfo devInfo_;
    rapidjson::Document netConfigDoc_;
    hj_bf::HJSubscriber wifiConnSub_;
    hj_bf::HJSubscriber devInfoReportSub_;
    hj_bf::HJPublisher wifiPub_;
    unsigned char aeskey_[16];
    uint8_t wifiSetFrom_;
    hj_bf::HJTimer resetFlagTmr_;

private:
    void wifiConnCallBack(const std_msgs::String& msg);
    void devInfoReportCallBack(const hj_interface::AppMsg::ConstPtr& msg);
    bool wifiParaDecode(const std::string& ssid, const std::string& pwd,
        const std::string& iv, std::string& ssiddec, std::string& pwddec);
    void awsCertWget();
    bool isCertFileExist();
    bool postGetCert(const std::string& url, const std::string& data, const std::string& token, 
        const std::string& encryptkey, const std::string& aeskey, const std::string& aesiv,
        std::string& res);
    bool awsCertStoreAndRunIot(const std::string& data);
    bool storeAwsFile(const std::string& data);
    void netConfigResp(int8_t code, const std::string& msg);
    void resetFlagTmrCallBack(const hj_bf::HJTimerEvent&);
    void certGetRepot();
    void sendErrorViaBt(int code, std::string msg = "");
};

} //namespace collect_node_iot