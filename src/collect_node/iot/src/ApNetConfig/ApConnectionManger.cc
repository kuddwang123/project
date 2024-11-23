#include "ApConnectionManager.h"
#include "Poll.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include "log.h"
#include "base64/base64.h"
#include "Utils.h"

namespace collect_node_iot {

const ApNetConfigManagerPtr& ApNetConfigManger::instance()
{
    static ApNetConfigManagerPtr ptr = std::make_shared<ApNetConfigManger>();
    return ptr;
}

void ApNetConfigManger::startSrvLoop(uint16_t port)
{
    if (srvTcpPtr_) {
        return;
    }

    srvTcpPtr_ = std::make_shared<TcpTransport>();
    assert(srvTcpPtr_);

    if (!srvTcpPtr_->listen(port, 10)) {
        HJ_ERROR("listen fail\n");
        return;
    }

    HJ_INFO("tcp listen on %d\n", port);

    expireTmr_ = hj_bf::HJCreateTimer("apExipre", 30 * 1000 * 1000, &ApNetConfigManger::expireTmrCb, this, false);
    
    Poll::instance()->loop(100);
}

void ApNetConfigManger::registerClient(const TcpTransportPtr& tcp)
{
    expireTmr_.start();
    std::unique_lock<std::mutex> lock(mtx_);
    clientTcpVec_.push_back(tcp);
    HJ_INFO("register client socket [%s]\n", tcp->id().data());
}

void ApNetConfigManger::expireTmrCb(const hj_bf::HJTimerEvent&)
{
    std::unique_lock<std::mutex> lock(mtx_);
    if (clientTcpVec_.empty()) {
        expireTmr_.stop();
        return;
    }

    for (auto it = clientTcpVec_.begin(); it != clientTcpVec_.end();) {
        if (!((*it)->isActive())) {
            HJ_INFO("erase tcp [%s]\n", (*it)->id().c_str());
            it = clientTcpVec_.erase(it);
            continue;
        }
        ++it;
    }
}

void ApNetConfigManger::dealDataReadFromAp(const std::string& data, const std::string& sessionId)
{
    std::string key, payload;
    
    std::unique_lock<std::mutex> lock(mtx_);
    auto it = std::find_if(clientTcpVec_.begin(), clientTcpVec_.end(), [&](const TcpTransportPtr& tcp){
        return tcp->id() == sessionId;
    });
    lock.unlock();

    if (it == clientTcpVec_.end()) {
        HJ_INFO("client [%s] not found\n", sessionId.c_str());
        return;
    } else {
        if (!((*it)->isActive())) {
            HJ_INFO("client is inactive, drop\n");
            return;
        }
    }

    if (!apDataParser(data, key, payload)) {
        return;
    }

    if (apDataJsonCb_) {
        apDataJsonCb_(key, payload, sessionId);
    }
}

bool ApNetConfigManger::apDataParser(const std::string in, std::string& rnout, std::string& plout)
{
    rapidjson::Document document;

    std::string base64DecodeString = base64_decode(in, true);
    std::string xorDecodeString = utils::xor_encrypt(base64DecodeString);
    std::string datakey;
    std::string datavalue;

    HJ_INFO("ap parse:\n%s\n", xorDecodeString.data());

    if (document.Parse(xorDecodeString.data()).HasParseError()) {
        HJ_ERROR("parse ap data fail");
        return false;
    }

    //Only support one pair key-value data
    for (rapidjson::Value::ConstMemberIterator itr = document.MemberBegin(); itr != document.MemberEnd(); ++itr) {
        //Ap may not have chksum
        if(strcmp(itr->name.GetString(), "chksum")) {
            datakey = itr->name.GetString();
            HJ_INFO("get ap key:%s\n", datakey.c_str());
            break;
        } 
    }

    if (datakey.empty()) {
        HJ_ERROR("not found ap data key\n");
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

    rnout = datakey;
    plout = datavalue;
    
    return true;
}

void ApNetConfigManger::apReport(const std::string& key, const std::string& payload, const std::string& session)
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

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer); 

    std::string value = buffer.GetString();
    HJ_INFO("ap report to [%s]:\n%s\n", session.c_str(), value.data());

    std::string xor_encry = utils::xor_encrypt(value);
    std::string base64_encry = base64_encode(xor_encry);
    base64_encry += '\n';

    if (session.empty()) {
        std::unique_lock<std::mutex> lock(mtx_);
        for (const auto& tcp: clientTcpVec_) {
            if (tcp->isActive()) {
                tcp->writeToBuf(base64_encry.data(), base64_encry.size());
            }
        }
    } else {
        std::unique_lock<std::mutex> lock(mtx_);
        for (const auto& tcp: clientTcpVec_) {
            if (tcp->id() == session && tcp->isActive()) {
                tcp->writeToBuf(base64_encry.data(), base64_encry.size());
            }
        }
    }
}

void ApNetConfigManger::apSend(const std::string& key, const std::string& payload, int8_t res, const std::string& sessionId)
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

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer); 

    std::string value = buffer.GetString();
    HJ_INFO("ap send:\n%s\n", value.c_str());

    std::string xor_encry = utils::xor_encrypt(value);
    std::string base64_encry = base64_encode(xor_encry);
    base64_encry += '\n';

    std::unique_lock<std::mutex> lock(mtx_);
    auto it = std::find_if(clientTcpVec_.begin(), clientTcpVec_.end(), [&](const TcpTransportPtr& tcp){
        return tcp->id() == sessionId;
    });
    lock.unlock();

    if (it != clientTcpVec_.end()) {
        HJ_INFO("write %ld bytes to client\n", base64_encry.size());
        (*it)->writeToBuf(base64_encry.data(), base64_encry.size());
    } else {
        HJ_ERROR("client not found for [%s]\n", sessionId.c_str());
    }
}

void ApNetConfigManger::closeClients()
{
    std::unique_lock<std::mutex> lock(mtx_);
    if (clientTcpVec_.empty()) {
        return;
    }

    for (auto it = clientTcpVec_.begin(); it != clientTcpVec_.end();) {
        HJ_INFO("positively erase tcp [%s]\n", (*it)->id().c_str());
        (*it)->close();
        it = clientTcpVec_.erase(it);
    }
}   

} //namespace collect_node_iot