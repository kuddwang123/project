#pragma once
#include "Poll.h"
#include "Buffer.h"

namespace collect_node_iot {

class TcpTransport;
typedef std::shared_ptr<TcpTransport> TcpTransportPtr;

class TcpTransport {
public:
    TcpTransport();
    ~TcpTransport();

    bool listen(uint16_t port, int backlog);
    void setSocket(int socket) {sockfd_ = socket;};
    void writeToBuf(const char* data, size_t len);
    bool isActive() const {return isActive_;}
    std::string id() const {return id_;}
    void close();
    
private:
    int sockfd_;
    bool isSrvSock_;
    bool isActive_;
    Poll* pollInst_;
    Buffer buffer_;
    std::string id_;

private:
    void socketUpdate(int events);
    void initClientSocket();
    void setSocketNonBlock();
    void setSocketReuse(bool);
    void setSocketKeepAlive(bool use);
    int accept();
    ssize_t read();
    ssize_t write();
    bool needOperLater();
    void closeClient();
};

} //namespace collect_node_iot;