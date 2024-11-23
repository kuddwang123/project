#pragma once

#include "MessageQ.h"
#include "node_factory.h"
#include "std_msgs/String.h"
#include "hj_interface/FileUpload.h"
#include "hj_interface/AppOnlineType.h"
#include <condition_variable>
#include <thread>
namespace collect_node_posttunnel {

class Reporter {
public:
    explicit Reporter(uint32_t queueSize, uint32_t loadInterval);
    void start();
    void stop();
    void reset();
    
private:
    bool quit_;
    uint32_t loadIntervalSec_;
    bool isIotOnline_;
    hj_bf::HJSubscriber uploadfile_sub_;
    hj_bf::HJSubscriber url_sub_;
    hj_bf::HJSubscriber iot_online_sub_;
    hj_bf::HJPublisher cloud_pub_; 
    //hj_bf::HJPublisher upload_rst_pub_;
    MessageQ activeQ_;
    MessageQ failQ_;
    std::mutex mtx_;
    std::condition_variable cond_;
    std::thread msgQueDealThread_;
    std::string failMsgFile_;
    hj_bf::HJTimer loadFailTmr_;

private:
    void pubToGetUrl(const std::string& uuid, uint8_t type, 
        const std::string& filename, uint64_t logid = 0);
    void failQueueGetUrl();
    void runInThread();
    void loadFailTimerCb(const hj_bf::HJTimerEvent &);
    void uploadFileCallBack(const hj_interface::FileUpload::ConstPtr&);
    void urlCallBack(const std_msgs::String::ConstPtr&);
    void appOnlineCallBack(const hj_interface::AppOnlineType::ConstPtr&);
    void saveFailMsgs();
    void loadFailMsgs();
    void reportMsg(const FileMsgPtr& msg);
    void reportMsgInThread(const FileMsgPtr& msg);
    //void reportFileUploadRst(const std::string& filename, uint8_t type, bool result);
    std::vector<FileMsgPtr> readAllMessages();
    void reportCleanResult(const FileMsgPtr& msg);
    void checkCachedFile();
};

} //namespace collect_node_posttunnel