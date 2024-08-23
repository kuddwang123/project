#pragma once

#include "Defines.h"
#include <vector>
#include <sys/epoll.h>
#include <thread>
#include <mutex>

namespace collect_node_iot {

class Poll {
public:
    static Poll* instance();
    void loop(int timeout);
    void shutdown();
    bool addSocket(int fd, const socketUpdateFunc& updatecb);
    bool deleteSocket(int fd);

private:
    typedef struct socketInfo {
        int _fd;
        socketUpdateFunc _cbfunc;
        socketInfo(int fd, const socketUpdateFunc& fn):
            _fd(fd),
            _cbfunc(fn) {}
    }socketInfo;

    static Poll* instance_;
    std::mutex mtx_;
    bool isRun_;
    int epollfd_;
    int timeout_;
    epoll_event events_[5];
    std::thread loopthread_;    
    std::vector<std::shared_ptr<socketInfo>> sockVec_;

private:
    Poll();
    ~Poll();
    void loopThreadFunc();
    void update(int timeout);
};

} //namespace collect_node_iot