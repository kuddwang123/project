#include "AppCommunication.h"
#include "AwsConnectionManager.h"
#include "Persistence.h"
#include "BuryPoint.h"
#include "log.h"
#include "hj_interface/WifiSet.h"
#include "base64/base64.h"
#include "hj_interface/SysAction.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <sstream>

HJ_REGISTER_FUNCTION(factory) {
    HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
    factory.registerCreater<collect_node_iot::AppCommunication>(FUNCTION_NAME);
}

namespace collect_node_iot {

AppCommunication::AppCommunication(const rapidjson::Value& json_conf) 
    :hj_bf::Function(json_conf),
    certdir_("/userdata/.root/cert/"),
    certfile_(".cert.json"),
    tzmdir_("/userdata/.root/etc/"),
    tzmfile_("tzm.json"),
    port_(8888),
    inWater_(255)
{
    if (json_conf.HasMember("tcpPort") && json_conf["tcpPort"].IsInt()) {
        port_ = json_conf["tcpPort"].GetInt();
    }

    std::thread ([&]() {
        appDataRouterPtr_ = std::make_shared<AppDataRouter>(certdir_+certfile_);
        assert(appDataRouterPtr_);
        
        if (!boost::filesystem::is_directory(certdir_)) {
            if (!boost::filesystem::create_directories(certdir_)) {
                throw std::runtime_error("aws cert dir create fail!"); 
            } else {
                HJ_INFO("aws cert dir [%s] created successfully.", certdir_.c_str());
            }
        }

        if (!boost::filesystem::is_directory(tzmdir_)) {
            if (!boost::filesystem::create_directories(tzmdir_)) {
                throw std::runtime_error("aws cert dir create fail!"); 
            } else {
                HJ_INFO("timezone dir [%s] created successfully.", tzmdir_.c_str());
            }
        }
        
        certfilename_ = certdir_ + certfile_;

        initialize();

        BuryPointPtr burypoint = BaseBuryPointFactory::instance().createBuryPoint(BaseBuryPointFactory::kNET_CONFIG);
        assert(burypoint);
        NetCfgBuryPointPtr netbp = std::dynamic_pointer_cast<NetConfigBuryPoint>(burypoint);   
        netCfgPtr_ = std::make_unique<NetConfig>(appDataRouterPtr_, certfilename_, netbp);
        assert(netCfgPtr_);
     }).detach();
}

AppCommunication::~AppCommunication()
{

}

void AppCommunication::initialize()
{
    appDataRouterPtr_->initialize(port_);
    appDataRouterPtr_->setAppDataAnalyser(boost::bind(&AppCommunication::appDataHandler, this,
                                  boost::placeholders::_1));

    appDataSub_ = hj_bf::HJSubscribe("/to_collect", 1, &AppCommunication::middleWareDataCb, this);

    resetSub_ = hj_bf::HJSubscribe("collect_node/notify/func", 1, &AppCommunication::collectNodeResetCb, this);

    resetResPub_ = hj_bf::HJAdvertise<hj_interface::CollectBroadcast>("func/response/collect_node", 1);

    s3urlPub_ = hj_bf::HJAdvertise<std_msgs::String>("/s3url/response", 10);

    timePub_ = hj_bf::HJAdvertise<std_msgs::String>("/time/settime", 5);

    getLogPub_ = hj_bf::HJAdvertise<hj_interface::AppData>("/getLogRecord", 5);

    toMidPub_ = hj_bf::HJAdvertise<std_msgs::String>("/to_middle", 10);

    otaPub_ = hj_bf::HJAdvertise<hj_interface::AppMsg>("aiper_ota_trigger", 1);

    inwaterSub_ = hj_bf::HJSubscribe("/water_inspection", 1, &AppCommunication::inWaterCb, this);

    cmdProcessPub_ = hj_bf::HJAdvertise<hj_interface::AppData>("/cmd_process", 5);
    initIot();
}

void AppCommunication::initIot()
{
    if (isCertFileExist()) {
        HJ_INFO("cert file [%s] exist!\n", certfilename_.c_str());
        utils::AwsCertParser awsCertParser(certfilename_.data());
        if (awsCertParser.parseOk()) {
            if (appDataRouterPtr_->initIot(awsCertParser.cert(), 
                                           awsCertParser.prikey(), 
                                           awsCertParser.thingname(), 
                                           awsCertParser.endpoint(), 
                                           awsCertParser.thingname())) {
                appDataRouterPtr_->runIot(false);
            }
        } else {
            HJ_ERROR("cert parse [%s] error\n", certfilename_.c_str());
            return;
        }
    } else {
        HJ_INFO("cert file [%s] not exist\n", certfilename_.c_str());
    }
}

void AppCommunication::inWaterCb(const std_msgs::UInt8::ConstPtr& msg)
{
    HJ_INFO("receive in water:%d", msg->data);

    if (inWater_ == 0 && msg->data == 1) {
        appDataRouterPtr_->restartIot();
    }
    inWater_ = msg->data;
}

void AppCommunication::collectNodeResetCb(const hj_interface::CollectBroadcast::ConstPtr& msg)
{
    HJ_INFO("iot receive reset, act:%d\n", msg->action);
    if (msg->action == hj_interface::SysAction::SYS_ACTION_RESTORE_FACTORY) {
        HJ_INFO("iot factory reset\n");
        iotFactoryReset();
        Persistence::instance().reset();
        HJ_INFO("iot factory reset done\n");
    } else if (msg->action == hj_interface::SysAction::SYS_ACTION_SHUTDOWN ||
            msg->action == hj_interface::SysAction::SYS_ACTION_SLEEP) {
        HJ_INFO("iot stop\n");
        appDataRouterPtr_->stopIot();
        Persistence::instance().shutdown();
        HJ_INFO("iot stop done\n");
    }

    hj_interface::CollectBroadcast response_msg;
    response_msg.action = msg->action;
    response_msg.ack = 0;
    response_msg.function = 0;
    resetResPub_.publish(response_msg);
}

void AppCommunication::iotFactoryReset()
{
    hj_interface::AppData appdata;
    hj_interface::AppMsg appmsg;
    rapidjson::Document rptDoc;
    std::remove(certfilename_.data());
    std::remove((tzmdir_+tzmfile_).data());
    HJ_INFO("cert removed\n");
    rptDoc.SetObject();
    appdata.key = "FactoryRestoreReport";
    appdata.payload = utils::documentToString(rptDoc);
    appmsg.appdata.emplace_back(appdata);
    appmsg.to = hj_interface::AppMsg::TOPIC | hj_interface::AppMsg::CLOUD;
    appDataRouterPtr_->sendAppRpt(appmsg);
    sleep(2);
    appDataRouterPtr_->unbindIot();
}

bool AppCommunication::appDataHandler(const hj_interface::AppMsg& appmsg)
{
    /*Here is analyse data from app*/

    hj_interface::AppMsg respmsg;
    respmsg.from = appmsg.from;
    respmsg.session = appmsg.session;
    size_t dealtcnt = 0;

    HJ_INFO("app data receive, size=%ld\n", appmsg.appdata.size());

    for (const auto& msg:appmsg.appdata) {
        if (msg.key == "FactoryTest") {
            rapidjson::Document respDoc;
            respDoc.SetObject();

            hj_interface::AppData appdata;
            appdata.key = "FactoryTest";
            appdata.payload = utils::documentToString(respDoc);
            respmsg.appdata.emplace_back(appdata);

            appDataRouterPtr_->sendAppResp(respmsg);

            dealtcnt++;
        } else if (msg.key == "NetStat") {
            HJ_INFO("NetStat receive!\n");
    
            rapidjson::Document respDoc;
            respDoc.SetObject();
            respDoc.AddMember("ble", 1, respDoc.GetAllocator());
            respDoc.AddMember("ap", 0, respDoc.GetAllocator());
            respDoc.AddMember("sta", 2, respDoc.GetAllocator());
            respDoc.AddMember("cert", isCertFileExist() ? 1 : 0, respDoc.GetAllocator());
            respDoc.AddMember("online", AwsConnectionManager::instance()->isConnected() ? 1 : 0, respDoc.GetAllocator());

            hj_interface::AppData appdata;
            appdata.key = "NetStat";
            appdata.payload = utils::documentToString(respDoc);
            respmsg.appdata.emplace_back(appdata);

            appDataRouterPtr_->sendAppResp(respmsg);

            dealtcnt++;
        } else if (msg.key == "TimeZoneSet") {
            HJ_INFO("TimeZoneSet receive!\n");
            rapidjson::Document config;
            int8_t res = 0;
            if (!config.Parse(msg.payload.data()).HasParseError()) {
                std::string timezone, time;
                if (config.HasMember("timeZone") && config["timeZone"].IsString()) {
                    timezone = config["timeZone"].GetString();
                }
                if (config.HasMember("time") && config["time"].IsString()) {
                    time = config["time"].GetString();
                }

                if (timezone.empty() || time.empty()) {
                    HJ_ERROR("parse TimeZoneSet fail\n");
                    res = -1;
                } else {
                    std::string datapub = timeZoneSet(time, timezone);
                    if (datapub.empty()) {
                        res = -1;
                    } else {
                        std_msgs::String data;
                        rapidjson::Document doc;
                        data.data = datapub;
                        timePub_.publish(data);
                        HJ_INFO("pub time set [%s]\n", datapub.c_str());

                        std::ofstream file(tzmdir_ + tzmfile_, std::ios::out | std::ios::trunc);
                        doc.SetObject();
                        doc.AddMember("tzm", rapidjson::Value(timezone.data(), doc.GetAllocator()).Move(), doc.GetAllocator());
                        std::string outdata = utils::documentToString(doc);
                        if (file.is_open()) {
                            file << outdata;
                            file.close();
                            HJ_INFO("write tzm file [%s] complete\n", tzmfile_.c_str());
                        } else {
                            HJ_ERROR("open [%s] fail\n", tzmfile_.c_str());
                        }
                    }
                }
            } else {
                res = -1;
                HJ_ERROR("TimeZoneSet json parse error\n");
            }

            rapidjson::Document respDoc;
            hj_interface::AppData appdata;
            respDoc.SetObject();
            
            appdata.key = "TimeZoneSet";
            appdata.res = res;
            appdata.payload = utils::documentToString(respDoc);
            respmsg.appdata.emplace_back(appdata);
            appDataRouterPtr_->sendAppResp(respmsg);
           
            dealtcnt++;
        } else if (msg.key == "NetConfig") {
            HJ_INFO("NetConfig receive!\n");
            rapidjson::Document config;
            int8_t res = 0;
            auto bp = BaseBuryPointFactory::instance().getBuryPoint(BaseBuryPointFactory::kNET_CONFIG);
            assert(bp);

            NetCfgBuryPointPtr netbp = std::dynamic_pointer_cast<NetConfigBuryPoint>(bp);
            netbp->recordStartTime();
            netbp->setRoute(appmsg.from);

            if (!config.Parse(msg.payload.data()).HasParseError()) {
                std::string cert, key, thingname, endpoint, msg;
                if (!netCfgPtr_->netConfig(config, appmsg.from, appmsg.session)) {
                    res = -1;
                    netbp->triggerBigDataRpt(false, NetConfigBuryPoint::kINTERNAL_INIT_FAIL);
                }
            } else {
                res = -1;
                netbp->triggerBigDataRpt(false, NetConfigBuryPoint::kJSON_PARSE_FAIL);
                HJ_ERROR("Netconfig json parse error\n");
            }

            rapidjson::Document respDoc;
            hj_interface::AppData appdata;
            respDoc.SetObject();
            
            appdata.key = "NetConfig";
            appdata.res = res;
            appdata.payload = utils::documentToString(respDoc);
            respmsg.appdata.emplace_back(appdata);
            appDataRouterPtr_->sendAppResp(respmsg);
           
            dealtcnt++;
        } else if (msg.key == "NetSwitch") {
            HJ_INFO("NetSwitch receive!\n");
            int8_t res = 0;
            rapidjson::Document config;
            if (!config.Parse(msg.payload.data()).HasParseError()) {
                std::string ssid, pwd, id;
                if (config.HasMember("ssid") && config["ssid"].IsString()) {
                    ssid = config["ssid"].GetString();
                }
                if (config.HasMember("passwd") && config["passwd"].IsString()) {
                    pwd = config["passwd"].GetString();
                }
                if (config.HasMember("id") && config["id"].IsString()) {
                    id = config["id"].GetString();
                }

                if (!netCfgPtr_->wifiSet(ssid, pwd, id, appmsg.from, appmsg.session)) {
                    res = -1;
                }
            } else {
                res = -1;
                HJ_ERROR("NetSwitch json parse error\n");
            }

            rapidjson::Document respDoc;
            hj_interface::AppData appdata;
            respDoc.SetObject();
            
            appdata.key = "NetSwitch";
            appdata.res = res;
            appdata.payload = utils::documentToString(respDoc);
            respmsg.appdata.emplace_back(appdata);
            appDataRouterPtr_->sendAppResp(respmsg);

            dealtcnt++;
        } else if (msg.key == "uploadFileUrl") {
            std_msgs::String pub;
            pub.data = msg.payload;
            s3urlPub_.publish(pub);
            dealtcnt++;
        } else if (msg.key == "UrlOta") {
            otaPub_.publish(appmsg);
            dealtcnt++;
        } else if (msg.key == "FactoryRestore") {
            HJ_INFO("FactoryRestore receive!\n");
            pubAppUnbindToMid();
            rapidjson::Document respDoc;
            respDoc.SetObject();
            hj_interface::AppData appdata;
            appdata.key = "FactoryRestore";
            appdata.payload = utils::documentToString(respDoc);
            respmsg.appdata.emplace_back(appdata);
            appDataRouterPtr_->sendAppResp(respmsg);
            sleep(2);
            iotFactoryReset();

            dealtcnt++;
        } else if (msg.key == "GetLogRecord" || msg.key == "PullDeviceLog") {
            HJ_INFO("%s receive!\n", msg.key.c_str());
            getLogPub_.publish(msg);
            if (msg.key == "PullDeviceLog") {
                dealtcnt++;
            }
        } else if (msg.key == "switchLog") {
            HJ_INFO("%s receive!\n", msg.key.c_str());
            cmdProcessPub_.publish(msg);
        }
    }

    if (dealtcnt == appmsg.appdata.size()) {
        return true;
    }

    return false;
}

bool AppCommunication::isCertFileExist()
{
    std::ifstream fileStream(certfilename_);
    return fileStream.is_open();
}

std::string AppCommunication::timeZoneSet(const std::string& timeStr, const std::string& timeZoneOffset)
{
    std::istringstream iss(timeStr);
    std::string dateStr, timePart;
    std::getline(iss, dateStr, ',');
    std::getline(iss, timePart, ',');

    boost::gregorian::date date(boost::lexical_cast<int>(dateStr.substr(0, 4)),
                   boost::lexical_cast<unsigned short>(dateStr.substr(5, 2)),
                   boost::lexical_cast<unsigned short>(dateStr.substr(8, 2)));

    std::istringstream time_iss(timePart);
    int hours = 0, minutes = 0, seconds = 0;
    char delimiter = 0;

    if (time_iss >> hours >> delimiter && delimiter == ':' &&
        time_iss >> minutes >> delimiter && delimiter == ':' &&
        time_iss >> seconds) {
        
        std::ostringstream oss;
        oss << "Hours: " << hours << ", Minutes: " << minutes << ", Seconds: " << seconds << std::endl;
        HJ_INFO("timezone set parse: %s\n", oss.str().c_str());
    } else {
        HJ_ERROR("Failed to parse time.\n");
        return "";
    }

    boost::posix_time::ptime originalTime(date, boost::posix_time::hours(hours) + 
                                                boost::posix_time::minutes(minutes) + 
                                                boost::posix_time::seconds(seconds));

    int offsetHours = 0;
    if (timeZoneOffset.find("+") != std::string::npos) {
        offsetHours = boost::lexical_cast<int>(timeZoneOffset.substr(timeZoneOffset.find('+') + 1));
    } else if (timeZoneOffset.find("-") != std::string::npos) {
        offsetHours = -boost::lexical_cast<int>(timeZoneOffset.substr(timeZoneOffset.find('-') + 1));
    }

    boost::posix_time::ptime utcTime = originalTime - boost::posix_time::hours(offsetHours);

    std::ostringstream oss;
    oss << utcTime.date().year() << '-'
        << utcTime.date().month().as_number() << '-'
        << utcTime.date().day().as_number() << ' '
        << utcTime.time_of_day().hours() << ':'
        << utcTime.time_of_day().minutes() << ':'
        << utcTime.time_of_day().seconds();

    return oss.str();
}

void AppCommunication::pubAppUnbindToMid()
{
    rapidjson::Document document;
    document.SetObject();

    rapidjson::Value emptyObj(rapidjson::kObjectType);
    
    document.AddMember("appUnbind", emptyObj, document.GetAllocator());

    std_msgs::String msg;
    msg.data = utils::documentToString(document);
    toMidPub_.publish(msg);
    HJ_INFO("to mid pub: %s\n", msg.data.c_str());
}

void AppCommunication::middleWareDataCb(const std_msgs::String& msg)
{

}

}
