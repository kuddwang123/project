#include "NetConfig.h"
#include "log.h"
#include "hj_interface/WifiSet.h"
#include "base64/base64.h"
#include <std_msgs/Bool.h>
#include <curl/curl.h>
namespace collect_node_iot {

NetConfig::NetConfig(const std::shared_ptr<AppDataRouter>& appRouterPtr, std::string certfile, const NetCfgBuryPointPtr& netbp):
    appRouterPtr_(appRouterPtr),
    state_(kNONE),
    certfile_(certfile),
    buryPointer_(netbp),
    devInfo_("/tmp/devInfo.json"),
    aeskey_{0x02, 0x46, 0xb9, 0xc0, 
           0x0a, 0xf1, 0xee, 0x10, 
           0xfa, 0x0c, 0xd0, 0x0d, 
           0x48, 0x00, 0x88, 0x20}
{
    netConfigDoc_.SetObject();
    wifiPub_ = hj_bf::HJAdvertise<hj_interface::WifiSet>("/wifiConfig", 1);
    iotReadyPub_ = hj_bf::HJAdvertise<std_msgs::Bool>("/wifi/iotConn", 1);
    wifiConnSub_ = hj_bf::HJSubscribe("/wifi/netconfig", 1, &NetConfig::wifiConnCallBack, this);
    devInfoReportSub_ = hj_bf::HJSubscribe("/devInfoReport", 1, &NetConfig::devInfoReportCallBack, this);
    resetFlagTmr_ = hj_bf::HJCreateTimer("netConfigTmr", 25 * 1000 * 1000, &NetConfig::resetFlagTmrCallBack, this, false);
}

bool NetConfig::wifiSet(const std::string& ssid, const std::string& passwd, 
    const std::string& iv, uint8_t from, const std::string& session)
{
    if (state_ != kNONE) {
        HJ_ERROR("current state busy:%d\n", state_);
        sendErrorRptImmedia(3014, from, session);
        return true;
    }

    wifiSetFrom_ = from;
    sockSession_ = session;
    state_ = kWIFICONFIG;

    std::string ssiddec;
    std::string psddec;

    if (!wifiParaDecode(ssid, passwd, iv, ssiddec, psddec)) {
        state_ = kNONE;
        return false;
    }

    appRouterPtr_->stopIot();
    hj_interface::WifiSet wifimsg;
    wifimsg.ssid = ssiddec;
    wifimsg.passwd = psddec;
    wifiPub_.publish(wifimsg);
    resetFlagTmr_.start();

    return true;
}

bool NetConfig::wifiParaDecode(const std::string& ssid, const std::string& passwd,
        const std::string& iv, std::string& ssidout, std::string& pwdout)
{
    if (ssid.empty() || iv.empty()) {
        return false;
    }

    int ivarrlen = iv.size()/2;
    unsigned char ivarr[ivarrlen] = {0};
    bool ret = false;
    if (!utils::hexStringToUnsignedCharArray(iv, ivarr, ivarrlen)) {
        HJ_ERROR("invalid iv:%s\n", iv.c_str());
        return false;
    }

    int ssidarrlen = ssid.size()/2;
    unsigned char ssidarr[ssidarrlen] = {0};
    if (!utils::hexStringToUnsignedCharArray(ssid, ssidarr, ssidarrlen)) {
        HJ_ERROR("invalid ssid:%s\n", ssid.c_str());
        return false;
    }

    if (!passwd.empty()) {
        int pwdarrlen = passwd.size()/2;
        unsigned char pwdarr[pwdarrlen] = {0};
        if (!utils::hexStringToUnsignedCharArray(passwd, pwdarr, pwdarrlen)) {
            HJ_ERROR("invalid passwd:%s\n", passwd.c_str());
            return false;
        }
        ret = utils::aesCbc128Decode(pwdarr, pwdarrlen, aeskey_, ivarr, pwdout);
        if (!ret) {
            HJ_ERROR("decode passwd fail\n");
            return false;
        }
    }

    ret = utils::aesCbc128Decode(ssidarr, ssidarrlen, aeskey_, ivarr, ssidout);
    if (!ret) {
        HJ_ERROR("decode ssid fail\n");
        return false;
    }

    HJ_INFO("ssid:%s, passwd:%s\n", ssidout.c_str(), pwdout.c_str());
       
    return true;
}

bool NetConfig::netConfig(const rapidjson::Document& config, uint8_t from, const std::string& session)
{
    if (state_ != kNONE) {
        HJ_ERROR("current state busy:%d\n", state_);
        sendErrorRptImmedia(3014, from, session);
        return true;
    }

    state_ = kNETCONFIG;
    wifiSetFrom_ = from;
    sockSession_ = session;
    netConfigDoc_.RemoveAllMembers();
    netConfigDoc_.CopyFrom(config, netConfigDoc_.GetAllocator());

    std::string ssid;
    std::string passwd;
    std::string iv;
    std::string ssiddec;
    std::string psddec;

    if (netConfigDoc_.HasMember("ssid") && netConfigDoc_["ssid"].IsString()) {
        ssid = netConfigDoc_["ssid"].GetString();
    }

    if (netConfigDoc_.HasMember("passwd") && netConfigDoc_["passwd"].IsString()) {
        passwd = netConfigDoc_["passwd"].GetString();
    }

    if (netConfigDoc_.HasMember("id") && netConfigDoc_["id"].IsString()) {
        iv = netConfigDoc_["id"].GetString();
    }

    if (!wifiParaDecode(ssid, passwd, iv, ssiddec, psddec)) {
        state_ = kNONE;
        return false;
    }

    hj_interface::WifiSet wifimsg;
    wifimsg.ssid = ssiddec;
    wifimsg.passwd = psddec;
    wifiPub_.publish(wifimsg);
    resetFlagTmr_.start();

    return true;
}

void NetConfig::wifiConnCallBack(const std_msgs::String& msg)
{
    rapidjson::Document doc;
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;

    if (doc.Parse(msg.data.data()).HasParseError()) {
        HJ_ERROR("parse wifi conn msg fail\n");
        return;
    }

    if (!doc.HasMember("res") || !doc["res"].IsInt())
        return;
    
    int res = doc["res"].GetInt();
    HJ_INFO("wifi conn result:%d\n", res);

    switch (state_) {
        case kNONE:
            HJ_INFO("not in config mode\n");
        break;
        
        case kNETCONFIG: {
            resetFlagTmr_.stop();
            if (res != 0) { //wifi set fail
                if (res == 2) {
                    buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::KWIFI_CONFIG_SSID_NOTFOUD);
                    sendErrorRpt(3003);
                } else if (res == 3) {
                    buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kWiFI_CONFIG_PASSWD_ERROR);
                    sendErrorRpt(3004);
                } else if (res == 4) {
                    buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kWIFI_CONFIG_IP_ALLOC_FAIL);
                    sendErrorRpt(3013);
                } else if (res == 5) {
                    buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kWIFI_CONFIG_WIFI_WEAK);
                    sendErrorRpt(3005);
                }else {
                    buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kWIFI_CONFIG_FAIL_UNKNOWN);
                    sendErrorRpt(3004);
                }
                netConfigResp(res, "wifi set fail");
                state_ = kNONE;
            } else {
                rapidjson::Document rptDoc;
                rptDoc.SetObject();
                rptDoc.AddMember("sta", res == 0 ? 2 : 1, rptDoc.GetAllocator());
                appdata.key = "NetStatReport";
                appdata.payload = utils::documentToString(rptDoc);
                appmsg.appdata.emplace_back(appdata);
                appmsg.to = wifiSetFrom_;
                appRouterPtr_->sendAppRpt(appmsg);
                
                if (wifiSetFrom_ == hj_interface::AppMsg::AP) {
                    ApNetConfigManger::instance()->closeClients();
                }
                std::thread getAwsCertThread = std::thread(&NetConfig::awsCertWget, this);
                getAwsCertThread.detach();
            }
        }
        break;
        
        case kWIFICONFIG: {
            resetFlagTmr_.stop();
            if (res != 0) { //wifi set fail
                if (res == 2) {
                    sendErrorRpt(3003);
                } else if (res == 3) {
                    sendErrorRpt(3004);
                } else if (res == 4) {
                    sendErrorRpt(3013);
                } else if (res == 5) {
                    sendErrorRpt(3005);
                } else {
                    sendErrorRpt(3004);
                }
                state_ = kNONE;
            } else {
                std::thread([&]() {
                    if(appRouterPtr_->runIot(true)) {
                        iotConnPub(true);
                    }else {
                        iotConnPub(false);
                        sendErrorRpt(3009);
                    }
                    state_ = kNONE;
                }).detach();
            }
        }
        break;

        default:
            ;
    }
}

void NetConfig::iotConnPub(bool state)
{
    HJ_INFO("iot conn pub:%d\n", state);
    std_msgs::Bool msg;
    msg.data = state;
    iotReadyPub_.publish(msg);
}

void NetConfig::devInfoReportCallBack(const hj_interface::AppMsg::ConstPtr& msg)
{
    auto appdata = msg->appdata.at(0);
    HJ_INFO("receive devinfo:\n%s\n", appdata.payload.c_str());
    devInfoJsonStr_ = appdata.payload;
}

void NetConfig::certGetRepot()
{
    HJ_INFO("report cert get!\n");
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    
    appdata.key = "NetStat";
    appdata.payload = "{\"cert\":1}";
    appmsg.appdata.push_back(appdata);
    appmsg.to = wifiSetFrom_;
    appRouterPtr_->sendAppRpt(appmsg);
}

void NetConfig::awsCertWget()
{
    if (!((netConfigDoc_.HasMember("aeskey")) && netConfigDoc_["aeskey"].IsString()) ||
        !((netConfigDoc_.HasMember("aesiv")) && netConfigDoc_["aesiv"].IsString()) ||
        !((netConfigDoc_.HasMember("encryptkey")) && netConfigDoc_["encryptkey"].IsString()) ||
        !((netConfigDoc_.HasMember("token")) && netConfigDoc_["token"].IsString()) ||
        !((netConfigDoc_.HasMember("url")) && netConfigDoc_["url"].IsString()) ||
        !((netConfigDoc_.HasMember("timestamp")) && netConfigDoc_["timestamp"].IsString())) {

        HJ_ERROR("netconfig json invalid\n");
        buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kNETCONFIG_JSON_VALUE_INVALID);
        netConfigResp(-1, "netconfig json invalid");
        state_ = kNONE;
        iotConnPub(false);
        return;
    }

    std::string aeskey = netConfigDoc_["aeskey"].GetString();
    std::string aesiv = netConfigDoc_["aesiv"].GetString();
    std::string encryptkey = netConfigDoc_["encryptkey"].GetString();
    std::string token = netConfigDoc_["token"].GetString();
    std::string url = netConfigDoc_["url"].GetString();
    std::string timestamp = netConfigDoc_["timestamp"].GetString();

    std::string curlres;
    rapidjson::Document dataDoc;
    dataDoc.SetObject();
    std::string ip = utils::getIpAddrString();

    dataDoc.AddMember("cert", this->isCertFileExist() ? 1 : 0, dataDoc.GetAllocator());
    dataDoc.AddMember("ip", rapidjson::Value(ip.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
    dataDoc.AddMember("timestamp", rapidjson::Value(timestamp.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
    if (!devInfoJsonStr_.empty()) {
        rapidjson::Document doc;
        if (!doc.Parse(devInfoJsonStr_.data()).HasParseError()) {
            std::string cloudver = doc["cloudver"].GetString();
            std::string model = doc["model"].GetString();
            std::string sn = doc["sn"].GetString();
            std::string socver = doc["socver"].GetString();
            std::string mac = doc["waddr"].GetString();
            std::string blename = doc["bleName"].GetString();
            rapidjson::Value& charge = doc["charge"].GetObject();
            utils::replaceSpaceWithUnderline(model);
            utils::replaceSpaceWithUnderline(blename);

            dataDoc.AddMember("version", rapidjson::Value(cloudver.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            dataDoc.AddMember("model", rapidjson::Value(model.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            dataDoc.AddMember("sn", rapidjson::Value(sn.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            dataDoc.AddMember("mac", rapidjson::Value(mac.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            dataDoc.AddMember("socver", rapidjson::Value(socver.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            dataDoc.AddMember("bleName", rapidjson::Value(blename.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            if (charge["hasCharge"].GetInt() == 1) {
                std::string chargever = charge["version"].GetString();
                dataDoc.AddMember("chargever", rapidjson::Value(chargever.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
            }
        } else {
            HJ_ERROR("parse devinfo json error\n");
        }
    } else if (devInfo_.openSucc()) {
        std::string model = devInfo_.model();
        std::string sn = devInfo_.sn();
        std::string mac = devInfo_.waddr();
        std::string version = "V" + devInfo_.version();
        std::string blename = devInfo_.bleName();
        utils::replaceSpaceWithUnderline(model);
        utils::replaceSpaceWithUnderline(blename);

        dataDoc.AddMember("model", rapidjson::Value(model.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
        dataDoc.AddMember("sn", rapidjson::Value(sn.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
        dataDoc.AddMember("mac", rapidjson::Value(mac.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
        dataDoc.AddMember("socver", rapidjson::Value(version.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
        dataDoc.AddMember("bleName", rapidjson::Value(blename.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
    }

    bool getCertRet = false;
    int retrycnt = 2;
    int curlErrCode = CURLE_OK;
    while (retrycnt) {
        rapidjson::Document postdata;
        unsigned char aeskey_plaint[aeskey.size()] = {0};
        unsigned char aesiv_plaint[aesiv.size()] = {0};
        
        postdata.SetObject();
        if (dataDoc.HasMember("nonce")) {
            dataDoc.RemoveMember("nonce");
        }
        dataDoc.AddMember("nonce", rapidjson::Value(utils::randomString(4).data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
        std::string datastr = utils::documentToString(dataDoc);
        HJ_INFO("post data raw:\n%s\n", datastr.data());

        utils::asciiStringToUnsignedCharArray(aeskey, aeskey_plaint, aeskey.size());
        utils::asciiStringToUnsignedCharArray(aesiv, aesiv_plaint, aesiv.size());
        
        std::string dataStrAesEncode = utils::aesCbc128Encode(datastr, aeskey_plaint, aesiv_plaint);
        if (dataStrAesEncode.empty()) {
            HJ_ERROR("aes encode fail\n");
            buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kINTERNAL_POST_FAIL);
            netConfigResp(-1, "aes encode for post data fail");
            iotConnPub(false);
            state_ = kNONE;
            return;
        }

        std::string dataStrBase64Enc = base64_encode(dataStrAesEncode);
        postdata.AddMember("data", rapidjson::Value(dataStrBase64Enc.data(), dataDoc.GetAllocator()).Move(), dataDoc.GetAllocator());
        std::string postdataStr = utils::documentToString(postdata);
        if (postGetCert(url, postdataStr, token, encryptkey, aeskey, aesiv, curlres, curlErrCode)) {
            getCertRet = true;
            break;
        } else {
            --retrycnt;
            HJ_ERROR("post get fail\n");
            continue;
        }
    }

    if (!getCertRet) {
        HJ_ERROR("post fail excees max times, give up\n");
        iotConnPub(false);
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, utils::Curl::getErrorByCode(curlErrCode));
        if (curlErrCode == CURLE_OPERATION_TIMEDOUT) {
            sendErrorRpt(3006);
        } else if (curlErrCode == CURLE_COULDNT_RESOLVE_HOST) {
            sendErrorRpt(3008);
        }
        
        netConfigResp(-1, "cert get fail");
        state_ = kNONE;
        return;
    }

    if (!awsCertStoreAndRunIot(curlres)) {
        iotConnPub(false);
        netConfigResp(-1, "run iot fail");
    } else {
        iotConnPub(true);
        netConfigResp(0, "");
    }

    state_ = kNONE;
    return;
}

bool NetConfig::postGetCert(const std::string& url, const std::string& data, const std::string& token, 
        const std::string& encryptkey, const std::string& aeskey, const std::string& aesiv,
        std::string& res, int& errcode)
{
    unsigned char aeskey_plaint[aeskey.size()] = {0};
    unsigned char aesiv_plaint[aesiv.size()] = {0};
    bool ret = false;
    utils::Curl curl;
    
    curl.setUrl(url);
    curl.setTimeout(10);
    curl.addHeader("Content-Type", "application/json");
    curl.addHeader("token", token);
    curl.addHeader("encryptkey", encryptkey);

    HJ_INFO("post url:\n%s\n", url.c_str());
    HJ_INFO("post data:\n%s\n", data.c_str());
   
    bool curlret = curl.post(data);
    if (!curlret) {
        errcode = curl.getErrorCode();
        return false;
    }

    std::string base64d;
    try { 
        base64d = base64_decode(curl.getResponse());
    } catch (std::exception& e) {
        HJ_ERROR("catch base64 decode exp:%s\n", e.what());
        HJ_ERROR("curl response:\n%s\n", curl.getResponseStr());
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "base64 decode response fail");
        return false;
    }
    memset(aeskey_plaint,0,aeskey.size());
    memset(aesiv_plaint,0,aesiv.size());
    utils::asciiStringToUnsignedCharArray(aeskey, aeskey_plaint, aeskey.size());
    utils::asciiStringToUnsignedCharArray(aesiv, aesiv_plaint, aesiv.size());

    ret = utils::aesCbc128Decode(reinterpret_cast<const unsigned char*>(base64d.data()), 
                                base64d.size(), aeskey_plaint, aesiv_plaint, res);
    
    if (!ret) {
        HJ_ERROR("curl decode fail\n");
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "curl res aes decode fail");
        return false;
    }
    
    HJ_INFO("curl res:\n%s\n", res.c_str());
    return true;
}

bool NetConfig::awsCertStoreAndRunIot(const std::string& data)
{
    rapidjson::Document doc;
    if (doc.Parse(data.data()).HasParseError()) {
        HJ_ERROR("curl res parse error!\n");
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "parse curl json response fail");
        return false;
    }

    if ((doc.HasMember("code")) && doc["code"].IsString()) {
        std::string code = doc["code"].GetString();
        if (code != "200") {
            if ((doc.HasMember("message")) && (doc["message"].IsString())) {
                std::string errmsg = doc["message"].GetString();
                buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, errmsg);
            } else {
                buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, code);
            }

            sendErrorRpt(3999, data);
            HJ_ERROR("aws cert response error, curl code:%s\n", code.c_str());
            return false;
        }
    } else {
        HJ_ERROR("curl res has no code!\n");
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "curl res has no code");
        return false;
    }

    if (!((doc.HasMember("data")) && doc["data"].IsObject())) {
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "curl json response has no data");
        HJ_ERROR("aws cert response json invalid\n");
        return false;
    }
    
    rapidjson::Value& awsdata = doc["data"].GetObject();
    utils::AwsCertParser awsCertParser(awsdata);

    if (!awsCertParser.parseOk()) {
        HJ_ERROR("cert parse json value error\n");
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "aws cert data parse error");
        return false;
    }

    std::string certstr = utils::documentToString(awsdata);
    if (!storeAwsFile(certstr)) {
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kAWS_CERTFILE_FAIL, "aws cert store error");
        return false;
    }

    certGetRepot();
    
    if (!appRouterPtr_->initIot(awsCertParser.cert(), 
                                   awsCertParser.prikey(), 
                                   awsCertParser.thingname(), 
                                   awsCertParser.endpoint(), 
                                   awsCertParser.thingname())) {
        HJ_INFO("initIot fail!\n");
        buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kIOT_CONNECT_FAIL, "aws iot init fail");
        return false;
    } else {
        if (!appRouterPtr_->runIot(true)) {
            HJ_INFO("run iot fail!\n");
            buryPointer_->triggerBigDataRptWithMsg(false, NetConfigBuryPoint::kIOT_CONNECT_FAIL, "aws iot connect fail");
            sendErrorRpt(3009);
            return false;
        }
    }

    return true;
}

bool NetConfig::storeAwsFile(const std::string& data)
{
    std::ofstream file(certfile_, std::ios::out | std::ios::trunc);

    if (file.is_open()) {
        file << data;
        file.close();
        HJ_INFO("store cert to [%s] success\n", certfile_.c_str());
        return true;
    } else {
        HJ_ERROR("open [%s] fail\n", certfile_.c_str());
        return false;
    }
}

bool NetConfig::isCertFileExist()
{
    std::ifstream fileStream(certfile_);
    return fileStream.is_open();
}

void NetConfig::resetFlagTmrCallBack(const hj_bf::HJTimerEvent&)
{
    if (state_ != kNONE) {
        HJ_INFO("net config reset state from:%d\n", state_);
        sendErrorRpt(3004);
        if (state_ == kNETCONFIG) {
            buryPointer_->triggerBigDataRpt(false, NetConfigBuryPoint::kWIFI_CONFIG_TIMEOUT);
        }
        iotConnPub(false);
        state_ = kNONE;
    }

    resetFlagTmr_.stop();
}

void NetConfig::netConfigResp(int8_t code, const std::string& msg)
{
    //TODO:暂时不需要在配网过程中给与响应
    return;

    rapidjson::Document respDoc;
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    respDoc.SetObject();
    
    if (!msg.empty()) {
        respDoc.AddMember("errcode", rapidjson::Value(msg.data(), respDoc.GetAllocator()).Move(), respDoc.GetAllocator());
    }

    appdata.key = "NetConfig";
    appdata.res = code;
    appdata.payload = utils::documentToString(respDoc);
    
    appmsg.from = wifiSetFrom_;
    appmsg.appdata.emplace_back(appdata);
    appRouterPtr_->sendAppResp(appmsg);
}

void NetConfig::sendErrorRpt(int code, std::string msg)
{
    rapidjson::Document rptDoc;
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    rptDoc.SetObject();
    rptDoc.AddMember("failCode", code, rptDoc.GetAllocator());
    if ((code == 3999) && !msg.empty()) {
        rptDoc.AddMember("serverResponse", rapidjson::Value(msg.data(), rptDoc.GetAllocator()).Move(), rptDoc.GetAllocator());
    }

    appdata.key = "NetConfigFail";
    appdata.payload = utils::documentToString(rptDoc);
    
    appmsg.to = wifiSetFrom_;
    appmsg.session = sockSession_;
    appmsg.appdata.emplace_back(appdata);
    appRouterPtr_->sendAppRpt(appmsg);
}

void NetConfig::sendErrorRptImmedia(int code, uint8_t from, const std::string& session)
{
    rapidjson::Document rptDoc;
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    rptDoc.SetObject();
    rptDoc.AddMember("failCode", code, rptDoc.GetAllocator());

    appdata.key = "NetConfigFail";
    appdata.payload = utils::documentToString(rptDoc);
    
    appmsg.to = from;
    appmsg.session = session;
    appmsg.appdata.emplace_back(appdata);
    appRouterPtr_->sendAppRpt(appmsg);
}

} //namespace collect_node_iot