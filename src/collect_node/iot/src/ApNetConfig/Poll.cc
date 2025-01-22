#include "Poll.h"
#include "log.h"
namespace collect_node_iot {

Poll* Poll::instance_ = nullptr;

Poll* Poll::instance()
{
    if (instance_ == nullptr) {
        instance_ = new Poll();
    }
    return instance_;
}

Poll::Poll():
    isRun_(false),
    epollfd_(epoll_create(5)),
    timeout_(0)
{

}

Poll::~Poll()
{
    shutdown();
    ::close(epollfd_);
    delete instance_;
}

void Poll::shutdown()
{
    isRun_ = false;
    if (loopthread_.joinable()) {
        loopthread_.join();
    }
}

void Poll::loop(int timeout)
{
    if (isRun_) {
        return;
    }
    isRun_ = true;
    timeout_ = timeout;
    loopthread_ = std::thread(&Poll::loopThreadFunc, this);
}

void Poll::loopThreadFunc()
{
    HJ_INFO("epoll loop...\n");
    while (isRun_) {
        update(timeout_);
    }
}

void Poll::update(int timeout)
{
    socketUpdateFunc func;
    int cnt = epoll_wait(epollfd_, events_, 5, timeout);
    if (cnt == -1) {
        HJ_ERROR("epoll error\n");
        return;
    }

    for (int i = 0; i < cnt; i++) {
        std::lock_guard<std::recursive_mutex> lock(r_mtx_);
        auto it = std::find_if(sockVec_.begin(), sockVec_.end(), [&](const std::shared_ptr<socketInfo>& item) {
            return events_[i].data.fd == item->_fd; 
        });

        if (it != sockVec_.end()) {
            func = (*it)->_cbfunc;
            if (func) {
                func(events_[i].events);
            }
        }
    }
}

bool Poll::addSocket(int fd, const socketUpdateFunc& updatecb)
{
    epoll_event event;
    event.data.fd = fd;
    event.events = EPOLLIN | EPOLLOUT | EPOLLHUP | EPOLLRDHUP;
    
    std::lock_guard<std::recursive_mutex> lock(r_mtx_);
    auto it = std::find_if(sockVec_.begin(), sockVec_.end(), [&](const std::shared_ptr<socketInfo>& item) {
        return fd == item->_fd; 
    });

    if (it != sockVec_.end()) {
        HJ_ERROR("socket exist: %d\n", fd);
        return false;
    }

    if (epoll_ctl(epollfd_, EPOLL_CTL_ADD, fd, &event) != 0) {
        HJ_ERROR("epoll add [%d] fail\n", fd);
        return false;
    }
    
    sockVec_.emplace_back(std::make_shared<socketInfo>(fd, updatecb));
    HJ_INFO("epoll add: %d\n", fd);
    
    return true;
}

bool Poll::deleteSocket(int fd)
{
    std::lock_guard<std::recursive_mutex> lock(r_mtx_);
    auto it = std::find_if(sockVec_.begin(), sockVec_.end(), [&](const std::shared_ptr<socketInfo>& item) {
        return fd == item->_fd; 
    });
    //lock.unlock();
    
    if (it == sockVec_.end()) {
        HJ_INFO("socket delete: [%d] not exist\n", fd);
        return true;
    }
    
    ::close(fd);
    epoll_ctl(epollfd_, EPOLL_CTL_DEL, (*it)->_fd, NULL);
    HJ_INFO("socket delete: [%d] success\n", (*it)->_fd);
    sockVec_.erase(it);

    return true;    
}

} // namespace collect_node_iot
