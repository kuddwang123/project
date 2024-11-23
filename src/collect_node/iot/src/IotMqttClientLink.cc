#include "IotMqttClientLink.h"
#include "boost/function.hpp"
#include <aws/crt/UUID.h>
#include <aws/crt/JsonObject.h>
#include "log.h"

namespace collect_node_iot {

IotMqttClientLink::IotMqttClientLink()
{

}

IotMqttClientLink::~IotMqttClientLink()
{
    HJ_INFO("IotMqttClientLink destruct call\n");
}

bool IotMqttClientLink::initialize(const std::shared_ptr<IotMqttClient>& iotMqttPtr)
{
    iotMqttClientPtr_ = iotMqttPtr;

    iotMqttClientPtr_->setAppMqttReqCb(boost::bind(&IotMqttClientLink::mqttMsgIncome, this,
                                 boost::placeholders::_1, boost::placeholders::_2));

    return true;
}

void IotMqttClientLink::mqttMsgIncome(const Aws::Crt::ByteBuf &msg, uint8_t route)
{
    Aws::Crt::String data(reinterpret_cast<char*>(msg.buffer), msg.len);

    HJ_INFO("receive [%d] mqtt msg:%s\n", route, data.c_str());

    Aws::Crt::JsonObject subdata(data);

    if (!subdata.WasParseSuccessful())
        return;
    
    Aws::Crt::Map<Aws::Crt::String, Aws::Crt::JsonView> view = subdata.View().GetAllObjects();

    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appmsg.from = route;

    for (auto it = view.begin(); it != view.end(); ++it) {
       if (it->first == "chksum") {
            continue;
       }
       appdata.key = std::string(it->first.data());
       appdata.payload = std::string(it->second.WriteCompact().data());
       appmsg.appdata.emplace_back(appdata);
    }

    if (mqttReqFunc_) {
        mqttReqFunc_(appmsg);
    } else {
        for(const auto& m: appmsg.appdata) {
            HJ_INFO("skip mqtt:%s", m.key.c_str());
        }
    }
}

bool IotMqttClientLink::iotMqttResp(const hj_interface::AppMsg::ConstPtr& msg)
{
    Aws::Crt::JsonObject resp;

    if (msg->appdata.size() == 1) { //should only have one group response
        Aws::Crt::JsonObject data(msg->appdata.at(0).payload.data());
        if (!data.WasParseSuccessful())
            return false;
        resp.WithObject(msg->appdata.at(0).key.data(), data);
        resp.WithInteger("res", msg->appdata.at(0).res);
    } else {
        HJ_ERROR("invalid resp size:%ld\n", msg->appdata.size());
        return false;
    }

    iotMqttClientPtr_->mqttResp(resp.View().WriteCompact(), msg->from);

    return true;
}

bool IotMqttClientLink::iotMqttReport(const hj_interface::AppMsg& msg, uint8_t type)
{
    if (!iotMqttClientPtr_) {
        HJ_INFO("mqtt client not construct!\n");
        return false;
    }

    Aws::Crt::JsonObject report;
    Aws::Crt::JsonObject jsonValueNull;
    jsonValueNull.AsNull();

    for (const auto& data:msg.appdata) {
        if (data.payload.empty()) {
            report.WithObject(data.key.data(), jsonValueNull);
        } else {
            Aws::Crt::JsonObject subdata(data.payload.c_str());
            if (!subdata.WasParseSuccessful()) {
                HJ_ERROR("payload data invalid:%s\n", data.payload.c_str());
                return false;
            }
            if (subdata.View().IsObject() || subdata.View().IsListType()) {
                report.WithObject(data.key.data(), subdata);
            } else {
                HJ_ERROR("payload not object:%s\n", data.payload.c_str());
                continue;
            }
        }
    }

    switch(type) {
        case hj_interface::AppMsg::SHADOW:
            iotMqttClientPtr_->updateShadowReport(report.View().WriteCompact());
            break;

        case hj_interface::AppMsg::TOPIC:
            iotMqttClientPtr_->mqttAppReport(report.View().WriteCompact());
            break;
        
        case hj_interface::AppMsg::CLOUD:
            iotMqttClientPtr_->mqttCloudReport(report.View().WriteCompact());
            break;
        
        case hj_interface::AppMsg::BIGDATA:
            iotMqttClientPtr_->mqttBigDataReport(report.View().WriteCompact());
            break;
        
        default:
            return false;
    }
    
    return true;
}

#if 0
void IotMqttClientLink::iotMqttUpdateShadow(const std::string& data)
{
    Aws::Crt::String str(data.data());

    iotMqttClientPtr_->updateShadowReport(str);

    return;
}
#endif

}