#pragma once
#include <vector>
#include <boost/function.hpp>
#include "TcpTransport.h"
#include <mutex>
#include "node_factory.h"

namespace collect_node_iot {
class ApNetConfigManger;
typedef std::shared_ptr<ApNetConfigManger> ApNetConfigManagerPtr;
typedef boost::function<void(const std::string&, const std::string&, const std::string&)> apDataJsonCb;

class ApNetConfigManger {
typedef boost::function<void(const std::string&, const std::string&, const void*)> ApDataJsonCb;

public:
    static const ApNetConfigManagerPtr& instance();
    void startSrvLoop(uint16_t port);
    void apSend(const std::string& key, const std::string& payload, int8_t res, const std::string& sessionId);
    void apReport(const std::string& key, const std::string& payload, const std::string& session);
    void dealDataReadFromAp(const std::string& data, const std::string& sessionId);
    void registerClient(const TcpTransportPtr& tcp);
    void setApDataHandler(const apDataJsonCb& cb) {apDataJsonCb_ = cb;}
    void closeClients();

private:
    std::mutex mtx_;
    hj_bf::HJTimer expireTmr_;
    apDataJsonCb apDataJsonCb_;
    TcpTransportPtr srvTcpPtr_;
    std::vector<TcpTransportPtr> clientTcpVec_;

private:
    bool apDataParser(const std::string in, std::string& rnout, std::string& plout);
    void expireTmrCb(const hj_bf::HJTimerEvent&);
};
} //namespace collect_node_iot