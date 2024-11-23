#include "Reporter.h"
#include "Curl.h"
#include "log.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/istreamwrapper.h"
#include "hj_interface/AppMsg.h"
#include <dirent.h>

namespace collect_node_posttunnel {
Reporter::Reporter(uint32_t queuesize, uint32_t loadInterval):
    quit_(false),
    loadIntervalSec_(loadInterval),
    isIotOnline_(false),
    failQ_(queuesize),
    failMsgFile_(FileMessage::cacheDir + "failMsg.json")
{
    if (!boost::filesystem::is_directory(FileMessage::cacheDir)) {
        assert(boost::filesystem::create_directories(FileMessage::cacheDir));
    }
}

void Reporter::start()
{
    uploadfile_sub_ = hj_bf::HJSubscribe("/upload/file", 50, &Reporter::uploadFileCallBack, this);
    
    url_sub_ = hj_bf::HJSubscribe("/s3url/response", 10, &Reporter::urlCallBack, this);
    
    cloud_pub_ = hj_bf::HJAdvertise<hj_interface::AppMsg>("/ReportApp", 10);

    //upload_rst_pub_ = hj_bf::HJAdvertise<hj_interface::FileUploadResult>("/upload/file/result", 10);
    iot_online_sub_ =  hj_bf::HJSubscribe("/AppOnline", 1, &Reporter::appOnlineCallBack, this);

    loadFailTmr_ = hj_bf::HJCreateTimer("posttunnl/loadfail", loadIntervalSec_ * 1000 * 1000, &Reporter::loadFailTimerCb, this);
    
    std::thread([&](){loadFailMsgs();}).detach(); 

    msgQueDealThread_ = std::thread(&Reporter::runInThread, this);
}

void Reporter::stop()
{
    quit_ = true;
    cond_.notify_all();

    if (msgQueDealThread_.joinable()) {
        msgQueDealThread_.join();
    }
}

void Reporter::reset()
{
    this->stop();
    for (const auto& filemsg:failQ_) {
        filemsg->getCacheHandler().remove();
        HJ_INFO("[%s] reset!\n", filemsg->getCacheHandler().getFullNameWithPath().c_str());
    }
    std::deque<FileMsgPtr> empty;
    failQ_.swapWithOther(empty);
    activeQ_.swapWithOther(empty);
    saveFailMsgs();
    quit_ = false;
    msgQueDealThread_ = std::thread(&Reporter::runInThread, this);
}

void Reporter::uploadFileCallBack(const hj_interface::FileUpload::ConstPtr& msg)
{
    std::string filepath = msg->filePath;
    HJ_INFO("receive file upload [%d] [%s]\n", msg->type, filepath.c_str());

    File fileHandler(filepath);
    if (!fileHandler.isAbsolutePath() || !fileHandler.isFileExist()) {
        HJ_INFO("file [%s] invalid, drop!\n", filepath.c_str());
        return;
    }

    auto file = failQ_.getByFilePath(filepath, true);
    if (file) {
        HJ_INFO("file [%s] exist in fail queue, remove cache!\n", filepath.c_str());
        file->getCacheHandler().remove();
    }

    std::shared_ptr<FileMessage> filemessagePtr = 
        std::make_shared<FileMessage>(fileHandler, msg->type, msg->deleteOnSuccess);
    assert(filemessagePtr);
    
    if (msg->type == hj_interface::FileUpload::DEVICELOG) {
        filemessagePtr->setLogId(msg->logId);
    }

    if (!filemessagePtr->getFileHandler().copyFileToOther(filemessagePtr->getCacheHandler())) {
        HJ_ERROR("copy from [%s] to [%s] fail\n", filemessagePtr->getFileHandler().getFullNameWithPath().c_str(),
            filemessagePtr->getCacheHandler().getFullNameWithPath().c_str());
        return;
    }

    if (!filemessagePtr->equalOriginCached()) {
        HJ_ERROR("md5 not equal after copy!\n");
        filemessagePtr->getCacheHandler().remove();
        return;
    }

    failQ_.pushBack(filemessagePtr);
    
    saveFailMsgs();

    loadFailTmr_.start();

    if (isIotOnline_) {
        pubToGetUrl(filemessagePtr->getId(), filemessagePtr->getType(),
            filemessagePtr->getFileHandler().getFileName(), filemessagePtr->getLogId());
    }
} 

void Reporter::appOnlineCallBack(const hj_interface::AppOnlineType::ConstPtr& msg)
{
    if (!isIotOnline_ && (msg->type == hj_interface::AppOnlineType::IOT || 
            msg->type == hj_interface::AppOnlineType::BT_IOT)) {
        HJ_INFO("set posttunnel online true\n");
        isIotOnline_ = true;
        failQueueGetUrl();
    } else if (msg->type == hj_interface::AppOnlineType::OFFLINE || 
            msg->type == hj_interface::AppOnlineType::BT) {
        HJ_INFO("set posttunnel online false\n");
        isIotOnline_ = false;
    }
}

void Reporter::pubToGetUrl(const std::string& uuid, uint8_t type, const std::string& filename, uint64_t logid)
{
    rapidjson::Document doc;
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

    doc.SetObject();

    doc.AddMember("type", type, doc.GetAllocator());
    doc.AddMember("uuid", rapidjson::Value(uuid.data(), doc.GetAllocator()).Move(), doc.GetAllocator());
    doc.AddMember("file", rapidjson::Value(filename.data(), doc.GetAllocator()).Move(), doc.GetAllocator());
    if (type == hj_interface::FileUpload::DEVICELOG) {
        doc.AddMember("controlLogId", logid, doc.GetAllocator());
    }
    doc.Accept(writer);

    hj_interface::AppMsg appPub;
    hj_interface::AppData appData;

    appData.key = "uploadFileUrl";
    appData.payload = buffer.GetString();

    appPub.appdata.push_back(appData);
    appPub.to = hj_interface::AppMsg::CLOUD;
    cloud_pub_.publish(appPub);
}

void Reporter::saveFailMsgs()
{
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Value dataArray(rapidjson::kArrayType);
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    std::ofstream failMsgFile(failMsgFile_, std::ios::out | std::ios::trunc);

    if (!failMsgFile.is_open()) {
        HJ_ERROR("open file [%s] error\n", failMsgFile_.c_str());
        return;
    }

    {
        std::unique_lock<std::mutex> lock(mtx_);
        for (const auto& msg: failQ_) {
            rapidjson::Value subObject(rapidjson::kObjectType);
            subObject.AddMember("id", rapidjson::Value(msg->getId().data(), doc.GetAllocator()).Move(), doc.GetAllocator());
            subObject.AddMember("type", msg->getType(), doc.GetAllocator());
            subObject.AddMember("ts", static_cast<int64_t>(msg->getTimeStamp()), doc.GetAllocator());
            subObject.AddMember("delete", msg->needDeleteOrigin() ? 1 : 0, doc.GetAllocator());
            subObject.AddMember("filepath", rapidjson::Value(msg->getFileHandler().getFullNameWithPath().data(), doc.GetAllocator()).Move(), doc.GetAllocator());
            subObject.AddMember("cachefile", rapidjson::Value(msg->getCacheHandler().getFullNameWithPath().data(), doc.GetAllocator()).Move(), doc.GetAllocator());
            //subObject.AddMember("url", rapidjson::Value(msg->getUrl().data(), doc.GetAllocator()).Move(), doc.GetAllocator());
            subObject.AddMember("md5", rapidjson::Value(msg->getMd5().data(), doc.GetAllocator()).Move(), doc.GetAllocator());
            //subObject.AddMember("expire", msg->getExpire(), doc.GetAllocator());
            //subObject.AddMember("urlts", static_cast<int64_t>(msg->getUrlGenTs()), doc.GetAllocator());
            subObject.AddMember("controlLogId", msg->getLogId(), doc.GetAllocator());
            dataArray.PushBack(subObject, doc.GetAllocator());
        }

        doc.AddMember("data", dataArray, doc.GetAllocator());
        doc.Accept(writer);

        failMsgFile << buffer.GetString();
        failMsgFile.close();
    }
}

void Reporter::urlCallBack(const std_msgs::String::ConstPtr& msg)
{
    HJ_INFO("receive s3 url response:%s\n", msg->data.c_str());

    rapidjson::Document doc;
    if (doc.Parse(msg->data.data()).HasParseError()) {
        HJ_ERROR("parse s3 url response error\n");
        return;
    }

    if (!doc.IsObject()) {
        return;
    }

    std::string uuid = doc["uuid"].GetString();
    std::string url = doc["url"].GetString();
    int32_t expire = doc["urlExpire"].GetInt();

    auto filemsg = failQ_.getById(uuid);

    if (!filemsg) {
        HJ_ERROR("msg for id [%s] not exist\n", uuid.c_str());
        return;
    }

    filemsg->setUrl(url);
    filemsg->setUrlExpire(expire);
    filemsg->setUrlGenTs(::time(NULL));

    activeQ_.pushBack(filemsg);
    cond_.notify_one();
}

std::vector<FileMsgPtr> Reporter::readAllMessages()
{
    std::vector<FileMsgPtr> vecData;
    rapidjson::Document document;
    {
        std::unique_lock<std::mutex> lock(mtx_);
        std::ifstream ifs(failMsgFile_);
        if (!ifs.is_open()) {
            HJ_INFO("Could not open file [%s]!\n", failMsgFile_.c_str());
            return {};
        }

        rapidjson::IStreamWrapper isw(ifs);
        document.ParseStream(isw);

        if (document.HasParseError()) {
            HJ_ERROR("Error parsing JSON file [%s]!\n", failMsgFile_.c_str());
            return {};
        }
    }

    const rapidjson::Value& dataArray = document["data"];
    for (rapidjson::SizeType i = 0; i < dataArray.Size(); i++) {
        const rapidjson::Value& obj = dataArray[i];
        std::string id = obj["id"].GetString();
        int64_t ts = obj["ts"].GetInt64();
        uint8_t need_delete = obj["delete"].GetUint();
        std::string filepath = obj["filepath"].GetString();
        std::string filepathca = obj["cachefile"].GetString();
        //std::string url = obj["url"].GetString();
        std::string md5 = obj["md5"].GetString();
        //uint32_t expire = obj["expire"].GetInt();
        //int64_t urlts = obj["urlts"].GetInt64();
        uint8_t type = obj["type"].GetUint();
        uint64_t logid = 0;
        if (type == hj_interface::FileUpload::DEVICELOG) {
            logid = obj["controlLogId"].GetInt64();
        }
        
        auto filemsg = std::make_shared<FileMessage>(
            id, type, need_delete, filepath, filepathca, ts, md5);
        assert(filemsg);

        if (type == hj_interface::FileUpload::DEVICELOG) {
            filemsg->setLogId(logid);
        }
        vecData.emplace_back(filemsg);
    }

    return vecData;
}

void Reporter::failQueueGetUrl()
{
    if (failQ_.isEmpty()) {
        return;
    }

    {
        std::unique_lock<std::mutex> lock(mtx_);
        for (const auto& failmsg: failQ_) {
            if (failmsg->getUrl().empty() || failmsg->isUrlExpired()) {
                pubToGetUrl(failmsg->getId(), failmsg->getType(),
                    failmsg->getFileHandler().getFileName(), failmsg->getLogId());
            }
        }
    }
}

void Reporter::checkCachedFile()
{
    DIR *dir = ::opendir(FileMessage::cacheDir.data());
    if (dir == nullptr) {
        return;
    }

    struct dirent *entry;
    std::string filepath;
    bool find = false;
    while ((entry = ::readdir(dir)) != nullptr) {
        if (strcmp(entry->d_name, ".") != 0 && 
            strcmp(entry->d_name, "..") != 0 &&
            strcmp(entry->d_name, "failMsg.json") != 0) {
            HJ_ERROR("found extra cached file:%s, remove!\n", entry->d_name);
            find = true;
            filepath = FileMessage::cacheDir + std::string(entry->d_name);
            boost::filesystem::remove(boost::filesystem::path(filepath));
        }
    }

    closedir(dir);

    if (!find) {
        HJ_INFO("cached file check pass\n");
    } else {
        HJ_ERROR("cached file check fail\n");
        //throw std::runtime_error("posttunnel cached file check fail");
    }
}

void Reporter::loadFailMsgs()
{
    auto localdata = readAllMessages();
    HJ_INFO("load fail msg size:%ld\n", localdata.size());
    bool activeflag = false;
    bool removeflag = false;

    if (localdata.empty()) {
        HJ_INFO("read local data empty\n");
        loadFailTmr_.stop();
        checkCachedFile();
        return;
    }

    for (const auto& filemsg:localdata) {
        if (!filemsg->equalOriginCached()) {
            filemsg->getCacheHandler().remove();
            auto file = failQ_.getById(filemsg->getId(), true);
            removeflag = true;
            HJ_INFO("[%s] drop!\n", filemsg->getFileHandler().getFullNameWithPath().c_str());
            continue;
        }

        FileMsgPtr tmp = failQ_.getById(filemsg->getId());
        
        if (tmp) {
            if (!tmp->isUrlExpired() && tmp->needPersistent()) {
                tmp->clearFailTimes();
                activeQ_.pushBack(tmp);
                activeflag = true;
                HJ_INFO("msg [%s] not expire, continue upload\n", tmp->getId().c_str());
            } else if (!tmp->getUrl().empty() && tmp->isUrlExpired()) {
                tmp->clearUrl();
                if (isIotOnline_) {
                    pubToGetUrl(tmp->getId(), tmp->getType(),
                        tmp->getFileHandler().getFileName(), tmp->getLogId());
                    HJ_INFO("msg [%s] expired, get url again\n", tmp->getId().c_str());
                } else {
                    HJ_INFO("msg [%s] expired, iot not online\n", tmp->getId().c_str());
                }
            } else if (tmp->getUrl().empty() ){
                HJ_INFO("msg [%s] url empty, get url\n", tmp->getId().c_str());
                if (isIotOnline_) {
                    pubToGetUrl(tmp->getId(), tmp->getType(),
                        tmp->getFileHandler().getFileName(), tmp->getLogId());
                }
            } else {
                HJ_INFO("msg [%s] exists in local fail, skip\n", tmp->getId().c_str());
            }
            continue;
        } else {
            failQ_.pushBack(filemsg);
        }
    }

    if (removeflag) {
        saveFailMsgs();
    }

    if (activeflag) {
        cond_.notify_one();
    }
}

void Reporter::loadFailTimerCb(const hj_bf::HJTimerEvent &)
{
    loadFailMsgs();
}

#if 0
void Reporter::reportFileUploadRst(const std::string& filename, uint8_t type, bool result)
{

    hj_interface::FileUploadResult msg;
    msg.result = result;
    msg.filePath = filename;
    msg.type = type;
    upload_rst_pub_.publish(msg);
    HJ_INFO("file upload result pub: %s, %d, %d\n", filename.c_str(), type, result);
}
#endif

void Reporter::reportMsg(const FileMsgPtr& msg)
{
    std::thread th = std::thread(&Reporter::reportMsgInThread, this, msg);
    th.detach();
}

void Reporter::reportMsgInThread(const FileMsgPtr& msg)
{
    Curl curl;
    //curl.setTimeout(120);
    //curl.setConnectTimeout(10);
    curl.addHeader("Content-Length", std::to_string(msg->getFileHandler().getFileSize()));
    curl.setUrl(msg->getUrl());
    curl.put(msg->getCacheHandler().getFullNameWithPath(), msg->getCacheHandler().getFileSize());

    auto respcode = curl.getResponseCode();
    if (respcode == 200) {
        HJ_INFO("[%s] upload success!\n", msg->getFileHandler().getFullNameWithPath().c_str());
        if (msg->getType() == hj_interface::FileUpload::CLEANRESULT) {
            reportCleanResult(msg);
        }
        msg->getCacheHandler().remove();
        if (msg->needDeleteOrigin()) {
            msg->getFileHandler().remove();
        }
    } else {
        HJ_INFO("[%s] upload fail:%ld!\n", msg->getFileHandler().getFullNameWithPath().c_str(), curl.getResponseCode());
        msg->incrFailTimes();
        if (!msg->needPersistent()) {
            activeQ_.pushBack(msg);
            cond_.notify_one();
        }
    }
}

void Reporter::runInThread()
{
    std::mutex mtx;
    
    while (!quit_) {
        std::deque<FileMsgPtr> msgQueue;
        {
            std::unique_lock<std::mutex> lock(mtx); //This mutex is local, because MessageQ has lock inside
            cond_.wait(lock, [&]{
                return !activeQ_.isEmpty() || quit_;
            });

            activeQ_.swapWithOther(msgQueue);
            HJ_INFO("after swap, active size:%ld, queue size:%ld\n", msgQueue.size(), activeQ_.getSize());
        }

        if (quit_) {
            //failQ_ += activeQ_;
            saveFailMsgs();
            break;
        }

        /*
        auto msg = activeQ_.popFront();
        if (!msg->equalOriginCached()) {
            msg->getCacheHandler().remove();
            continue;
        }

        if (msg->isUrlExpired()) {
            msg->clearUrl();
            failQ_.pushBack(msg);
            continue;
        }
        
        reportMsg(msg);
        */
        
        for (const auto& msg: msgQueue) {
            if (!msg->equalOriginCached()) {
                msg->getCacheHandler().remove();
                continue;
            }

            if (msg->isUrlExpired()) {
                HJ_INFO("url expired while uploading for [%s]\n", msg->getId().c_str());
                continue;
            }

            reportMsg(msg);
        }
    }
}

void Reporter::reportCleanResult(const FileMsgPtr& msg)
{
    std::ifstream ifs(msg->getCacheHandler().getFullNameWithPath());
    if (!ifs.is_open()) {
        HJ_ERROR("Could not open file [%s]!\n", msg->getCacheHandler().getFullNameWithPath().c_str());
        return;
    }

    rapidjson::Document document;
    rapidjson::IStreamWrapper isw(ifs);
    document.ParseStream(isw);

    if (document.HasParseError()) {
        HJ_ERROR("Error parsing JSON file [%s]!\n", msg->getCacheHandler().getFullNameWithPath().c_str());
        return;
    }

    if (document.IsObject()) {
        if (document.HasMember("CleanRecord") && document["CleanRecord"].IsObject()) {
            const rapidjson::Value& cleanresult = document["CleanRecord"];
            rapidjson::Document doc;
            doc.CopyFrom(cleanresult, doc.GetAllocator());
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            doc.Accept(writer);

            hj_interface::AppMsg appPub;
            hj_interface::AppData appData;

            appData.key = "CleanRecord";
            appData.payload = buffer.GetString();
            appPub.appdata.push_back(appData);
            appPub.to = hj_interface::AppMsg::CLOUD;
            cloud_pub_.publish(appPub);
        } else {
            HJ_ERROR("parse \"CleanRecord\" fain\n");
            return;
        } 
    }
}

} //namespace collect_node_posttunnel