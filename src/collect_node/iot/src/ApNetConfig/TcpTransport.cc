#include "TcpTransport.h"
#include "ApConnectionManager.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "log.h"

namespace collect_node_iot {

TcpTransport::TcpTransport():
    sockfd_(-1),
    isSrvSock_(false),
    isActive_(true),
    pollInst_(Poll::instance())
{
    assert(pollInst_);
    boost::uuids::random_generator rgen;
    boost::uuids::uuid a_uuid = rgen();
    id_ = boost::uuids::to_string(a_uuid);
}

TcpTransport::~TcpTransport()
{
    HJ_INFO("tcp [%d] deconstruct\n", sockfd_);
    closeClient();
}

bool TcpTransport::listen(uint16_t port, int backlog)
{
    sockaddr_in serv_addr;
    bzero(&serv_addr, sizeof(sockaddr_in));

    sockfd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd_ < 0) {
        HJ_ERROR("new socket fail:%d\n", errno);
        return false;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    setSocketReuse(true);
    setSocketNonBlock();

    if(::bind(sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1) {
        HJ_ERROR("bind fail: %d\n", errno);
        return false;
    }

    if(::listen(sockfd_, backlog) == -1) {
        HJ_ERROR("listen fail: %d\n", errno);
        return false;
    }

    if (!pollInst_->addSocket(sockfd_, boost::bind(&TcpTransport::socketUpdate, 
            this, boost::placeholders::_1))) {
        
        HJ_ERROR("add socket [%d] fail\n", sockfd_);
        return false;
    }

    isSrvSock_ = true;
    return true;
}

int TcpTransport::accept()
{
    sockaddr_in cli_addr;
    socklen_t len = sizeof(sockaddr_in);
    bzero(&cli_addr, sizeof(cli_addr));

    int clisocket = -1;

    if ((clisocket = ::accept(sockfd_, (struct sockaddr *)&cli_addr, &len)) < 0) {
        HJ_ERROR("accept fail: %d\n", errno);
        return clisocket;
    }
    
    auto tcp = std::make_shared<TcpTransport>();
    tcp->setSocket(clisocket);

    HJ_INFO("accept client:%d\n", clisocket);

    pollInst_->addSocket(clisocket, boost::bind(&TcpTransport::socketUpdate, tcp,
        boost::placeholders::_1));

    ApNetConfigManger::instance()->registerClient(tcp);
    return clisocket;
}

ssize_t TcpTransport::read()
{
    char data[1024] = {0};
    ssize_t read_num = ::recv(sockfd_, data, sizeof(data), 0);
    if (read_num < 0) {
        if (needOperLater()) {
            HJ_INFO("read operate later [%d]\n", sockfd_);
            return 0;
        } else {
            HJ_INFO("read error [%d]: %d, close socket\n", sockfd_, errno);
            closeClient();
        }
    } else if (read_num == 0) {
        HJ_INFO("close socket [%d]\n", sockfd_);
        closeClient();
    } else {
        HJ_INFO("socket [%d] read size:%ld, data:%s\n", sockfd_, read_num, data);
        buffer_.read(data, read_num);
        if (buffer_.isReadDone()) {
            HJ_INFO("socket read complete\n");
            HJ_INFO("%d [%s]\n", sockfd_, id_.c_str());
            std::string rdata = std::move(buffer_.readData());
            ApNetConfigManger::instance()->dealDataReadFromAp(rdata, id_);
            buffer_.clearReadBuf();
        }
    }

    return read_num;
}

ssize_t TcpTransport::write()
{
    const uint8_t* wBegin = buffer_.writeBegin();
    size_t wLen = buffer_.writeIndex();
    if (*wBegin == 0 || wLen == 0) {
        return 0;
    }

    HJ_INFO("ready write %ld bytes to socket [%d]\n", wLen, sockfd_);
    ssize_t write_num = ::send(sockfd_, reinterpret_cast<const char*>(wBegin), wLen, 0);
    if (write_num < 0) {
        if (needOperLater()) {
            HJ_INFO("write operate later [%d]\n", sockfd_);
            return 0;
        } else {
            HJ_INFO("write error [%d]: %d, close socket\n", sockfd_, errno);
            closeClient();
        }
    } else if (write_num == 0) {
        HJ_INFO("close socket [%d]\n", sockfd_);
        closeClient();
    } else {
        if (write_num == wLen) {
            HJ_INFO("socket write %ld bytes complete!\n", write_num);
        } else if (write_num < wLen) {
            HJ_INFO("socket write %ld bytes, less than total %ld!\n", write_num, wLen);
        }
        buffer_.shrinkWriteBuf(write_num);
    }

    return write_num;
}

void TcpTransport::writeToBuf(const char* data, size_t len)
{
    buffer_.write(data, len);
}

void TcpTransport::setSocketNonBlock()
{
    int flags = ::fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0) {
        HJ_ERROR("fcntl F_GETFL failed\n");
        return;
    }

    flags |= O_NONBLOCK;
    if (::fcntl(sockfd_, F_SETFL, flags) < 0) {
        HJ_ERROR("fcntl F_SETFL failed\n");
    }
}

void TcpTransport::close()
{
    closeClient();
}

void TcpTransport::initClientSocket()
{
    setSocketNonBlock();
    setSocketKeepAlive(true);
}

void TcpTransport::setSocketReuse(bool opt)
{
    int optval = opt;
    ::setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
}

void TcpTransport::closeClient()
{
    isActive_ = false;
    pollInst_->deleteSocket(sockfd_);
}

void TcpTransport::setSocketKeepAlive(bool use)
{
    int val = -1;
    if (use) {
        val = 1;
        ::setsockopt(sockfd_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val));
    }
    else {
        val = 0;
        ::setsockopt(sockfd_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val));
    }
}

bool TcpTransport::needOperLater()
{
    if ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINPROGRESS)) {
		return true;
	} else {
		return false;
	}
}

void TcpTransport::socketUpdate(int events)
{
    //HJ_INFO("sock update:%d, events:%d, isServ:%d\n", sockfd_, events, isSrvSock_);
    if (isSrvSock_ && (events & EPOLLIN)) {
        int fd = this->accept();
        HJ_INFO("client connect, fd: %d\n", fd);
    } else if (events & EPOLLIN) {
        ssize_t len = this->read();
        if (len > 0) {
            HJ_INFO("read %ld bytes to client [%d]\n", len, sockfd_);
        }
    } else if (events & EPOLLOUT) {
        ssize_t len = this->write();
        if (len > 0) {
            HJ_INFO("write %ld bytes to client [%d]\n", len, sockfd_);
        }
    } else if (events & (EPOLLHUP | EPOLLRDHUP)) {
        HJ_INFO("client socket close!\n");
        closeClient();
    }
}

} //namespace collect_node_iot