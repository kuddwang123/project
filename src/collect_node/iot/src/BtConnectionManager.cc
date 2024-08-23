#include "BtConnectionManager.h"
#include <boost/bind.hpp>
#include <mutex>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "log.h"
#include "base64/base64.h"
#include "Utils.h"
#include <string.h>

namespace collect_node_iot {

BtConnectionManager::BtConnectionManager():
    isRun_(true),
    connState_(BT_IDLE)
{
}

BtConnectionManager::~BtConnectionManager() 
{
    isRun_ = false;
    cond_.notify_all();
}

const BtConnectionManagerPtr& BtConnectionManager::instance()
{
    static BtConnectionManagerPtr bt_connection_manager = 
        std::make_shared<BtConnectionManager>();
    return bt_connection_manager;
}

bool BtConnectionManager::isConnected() const 
{
    return connState_ == BT_CONN;
}

boost::signals2::connection BtConnectionManager::addConStatListener(const ConStaChangeCb& slot)
{
    return conchange_signal_.connect(slot);
}

bool BtConnectionManager::initialize() 
{
    bt_data_sub_ = hj_bf::HJSubscribe("bt/state", 1, &BtConnectionManager::btConnStateCb, this);

    bt_state_sub_ = hj_bf::HJSubscribe("bt/receive", 50, &BtConnectionManager::btDataRecvCb, this);

    bt_data_pub_ = hj_bf::HJAdvertise<std_msgs::String>("bt/send", 50);

    btDataParseThread_ = std::thread(&BtConnectionManager::btDataParseThread, this);
    btDataParseThread_.detach();
    
    return true;
}

void BtConnectionManager::btRpt(const std::string& key, const std::string& payload)
{
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document subdoc;
    rapidjson::Value subvalue;

    if (payload.empty()) {
        subvalue.SetNull();
    } else if (!subdoc.Parse(payload.data()).HasParseError() && subdoc.IsObject()) {
        subvalue.SetObject();
        subvalue.CopyFrom(subdoc, doc.GetAllocator());
    } else {
        return;
    }
    
    rapidjson::Value subkey(key.data(), doc.GetAllocator());
    doc.AddMember(subkey, subvalue, doc.GetAllocator());

    if (!payload.empty()) {
        uint16_t cs_cal = checksum_calculate(payload);
        doc.AddMember("chksum", cs_cal, doc.GetAllocator());
    }

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer); 

    std::string value = buffer.GetString();
    HJ_INFO("send:\n%s\n", value.data());

    std::string xor_encry = utils::xor_encrypt(value);
    std::string base64_encry = base64_encode(xor_encry);
    base64_encry += '\n';

    btMsgPub(base64_encry);
}

void BtConnectionManager::btSend(const std::string& key, const std::string& payload, int res)
{
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Document subdoc;

    if (subdoc.Parse(payload.data()).HasParseError() || !subdoc.IsObject())
        return;
    
    rapidjson::Value subvalue(rapidjson::kObjectType);
    rapidjson::Value subkey(key.data(), doc.GetAllocator());
    subvalue.CopyFrom(subdoc, doc.GetAllocator());
    doc.AddMember(subkey, subvalue, doc.GetAllocator());
    doc.AddMember("res", res, doc.GetAllocator());

    if (!subdoc.ObjectEmpty()) {
        uint16_t cs_cal = checksum_calculate(payload);
        doc.AddMember("chksum", cs_cal, doc.GetAllocator());
    }

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer); 

    std::string value = buffer.GetString();
    HJ_INFO("send:\n%s\n", value.data());

    std::string xor_encry = utils::xor_encrypt(value);
    std::string base64_encry = base64_encode(xor_encry);
    base64_encry += '\n';

    btMsgPub(base64_encry);
}

void BtConnectionManager::btConnStateCb(const std_msgs::String::ConstPtr& msg)
{
    //TODO:该话题发布时需要latch=true;
    if (msg->data.empty())
        return;

    rapidjson::Document document;
    if (document.Parse(msg->data.data()).HasParseError()) {
        HJ_ERROR("parse bt state fail:\n%s\n", msg->data.c_str());
        return;
    }

    if (document.HasMember("bt_state") && document["bt_state"].IsInt()) {
        int state = document["bt_state"].GetInt();
        HJ_INFO("blue tooth state change:%d\n", state);
        connState_ = static_cast<bt_conn_type>(state);
        if (state == 1)
            conchange_signal_(true);
        else
            conchange_signal_(false);
    }
}

bool BtConnectionManager::btDataParser(const std::string in, std::string& rnout, std::string& plout)
{
    rapidjson::Document document;

    std::string base64DecodeString = base64_decode(in, true);
    std::string xorDecodeString = utils::xor_encrypt(base64DecodeString);
    std::string datakey;
    std::string datavalue;

    HJ_INFO("bt parse:\n%s\n", xorDecodeString.data());

    if (document.Parse(xorDecodeString.data()).HasParseError()) {
        HJ_ERROR("parse bt data fail");
        return false;
    }

    //Only support one pair key-value data
    for (rapidjson::Value::ConstMemberIterator itr = document.MemberBegin(); itr != document.MemberEnd(); ++itr) {
        if(strcmp(itr->name.GetString(), "chksum")) {
            datakey = itr->name.GetString();
            HJ_INFO("get bt key:%s\n", datakey.data());
            break;
        } 
    }

    if (datakey.empty()) {
        HJ_ERROR("not found data key\n");
        return false;
    }

    if (!document[datakey.data()].IsObject()) {
        HJ_ERROR("[%s] not object\n", datakey.data());
        return false;
    }

    rapidjson::Value& obj = document[datakey.data()].GetObject();
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

    obj.Accept(writer); 
    datavalue = buffer.GetString();
#if 0
    if (document.HasMember("chksum") && document["chksum"].IsInt()) {
        int chksum = document["chksum"].GetInt();
        uint16_t cs_cal = checksum_calculate(datavalue);

        if (cs_cal != chksum) {
            HJ_ERROR("checksum cal error:%d, %d\n", chksum, cs_cal);
            return false;
        }
    }
#endif
    rnout = datakey;
    plout = datavalue;
    
    return true;
}

void BtConnectionManager::btDataRecvCb(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.empty())
        return;

    std::string income = msg->data;
    {
        std::unique_lock<std::mutex> lock(mtx_);
        dataBuf_ += income;
        cond_.notify_one();
    }       
}

void BtConnectionManager::btDataParseThread()
{
    while (isRun_) {
        std::unique_lock<std::mutex> lock(mtx_);
        cond_.wait(lock, [&] {
            return !isRun_ || !dataBuf_.empty();
        });

        if (dataBuf_.back() != '\n') {
            if (dataBuf_.size() >= MAX_BT_CACHE_LENGTH) {
                HJ_INFO("[WARN] bt data cache oversize:%ld\n", dataBuf_.size());
            }
            continue;
        }

        std::string buf;
        std::swap(buf, dataBuf_); 
        std::string key;
        std::string payload;
        bool ret = false;
        if (ret = btDataParser(buf, key, payload)) {
            //call func 
            //1. analyse is netconfig corresponding
            //2. pub /ReqFromApp
            if (btDataJsonCb_)
                btDataJsonCb_(key, payload);
        }
    }
}

void BtConnectionManager::btMsgPub(const std::string& msg)
{
    std_msgs::String pubdata;
    uint32_t count = msg.size()/BLUETOOTH_MTU + 1;
    if (count == 1) {
        pubdata.data = msg;
        bt_data_pub_.publish(pubdata);
    } else if (count > 1) {
        uint32_t sendlen = BLUETOOTH_MTU;
        for (uint32_t index = 0; index < count; index++) {
            sendlen = msg.size()-index*BLUETOOTH_MTU >= BLUETOOTH_MTU ? 
                      BLUETOOTH_MTU : 
                      msg.size()-index*BLUETOOTH_MTU;
            std::string sub = msg.substr(index*BLUETOOTH_MTU, sendlen);
            pubdata.data = sub;
            bt_data_pub_.publish(pubdata);
        }
    }
}

uint16_t BtConnectionManager::checksum_calculate(const std::string& data)
{
    static const uint8_t aucCRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40
    };

    static const uint8_t aucCRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
        0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
        0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
        0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
        0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
        0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
        0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
        0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
        0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
        0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
        0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
        0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
        0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
        0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
        0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
        0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
        0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
        0x41, 0x81, 0x80, 0x40
    };

    uint8_t ucCRCHi = 0x99;
    uint8_t ucCRCLo = 0x66;
    int iIndex;
    
    uint8_t byteArray[data.size()+1] = {0};

    for (size_t i = 0; i < data.size(); ++i) {
        byteArray[i] = static_cast<uint8_t>(data[i]);
    }

    
    for (size_t i = 0; i < sizeof(byteArray)/sizeof(uint8_t); ++i) {
        iIndex = ucCRCLo ^ byteArray[i];
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
/*
    uint32_t len = data.size();
    while( len-- ) {
        iIndex = ucCRCLo ^ *( byteArray++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
*/
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

}
