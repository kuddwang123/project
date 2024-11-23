#pragma once
#include "node_factory.h"
#include "AppDataRouter.h"
#include "hj_interface/AppOnlineType.h"
#include <boost/filesystem.hpp>
#include <pthread.h>
#include <map>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

namespace collect_node_iot {

class Persistence {
public:
    Persistence(const Persistence&) = delete;
    void operator=(const Persistence&) = delete;

    static Persistence& instance();
    void initialize(const AppDataRouterPtr& ptr);
    bool needPersistent(const std::string& key);
    void persistMsg(const std::string& key, const std::string& payload); 
    void reset();
    void shutdown();
    
private:
    static Persistence* instance_;
    static pthread_once_t ponce_;
    static std::string folderPrefix_;
    static std::vector<std::string> interestKey_;
    std::mutex mtx_;
    std::atomic<bool> appOnline_;
    std::atomic<bool> isRun_;
    std::map<std::string, std::deque<rapidjson::Document>> data_;
    hj_bf::HJSubscriber appOnlineSub_;
    hj_bf::HJTimer ckFileSizeTmr_;
    AppDataRouterPtr appRouterPtr_;
    std::thread refreshFileTh_;
    std::atomic<bool> refreshFlag_;

private:
    Persistence();
    ~Persistence();
    static void init();
    void loadJsonFile2Map(const boost::filesystem::path& filePath);
    void AppOnlineCb(const hj_interface::AppOnlineType::ConstPtr& msg);
    void refreshFileThread();
    void refreshAllData();
    void reportCacheMsg();
    bool isDataEmptyWithoutLock();
    void checkFileSizeTmrCb(const hj_bf::HJTimerEvent&);
    void decreaseHalfMsg();
    void pubIotMsg(const std::string& key, const std::string& payload, int type);
    bool refreshFile(const std::string& key, const std::deque<rapidjson::Document>&);
    uintmax_t getFolderSize();
};

}