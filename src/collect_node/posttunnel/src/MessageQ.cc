#include "MessageQ.h"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "log.h"
namespace collect_node_posttunnel {

std::string FileMessage::cacheDir = "/userdata/.postCache/";

FileMessage::FileMessage(const File& file, uint8_t type, bool needDelete):
    timestamp_(::time(NULL)),
    type_(type),
    needDelete_(needDelete),
    file_(file),
    cachedFile_(cacheDir + file_.getFileName()),
    expireSec_(0),
    urlTs_(0),
    failTimes_(0),
    maxTimes_(3)
{
    boost::uuids::random_generator rgen;
    boost::uuids::uuid a_uuid = rgen();
    id_ = boost::uuids::to_string(a_uuid);
    md5_ = file_.getMd5();
}   

FileMessage::FileMessage(const std::string id,
    uint8_t type,
    bool needDelete, 
    const std::string& filepath,
    const std::string& filepathca,
    time_t ts,
    const std::string& url,
    const std::string& md5,
    uint32_t urlexpire,
    time_t urlTs
    ):
    id_(id),
    timestamp_(ts),
    type_(type),
    needDelete_(needDelete),
    file_(filepath),
    cachedFile_(filepathca),
    expireSec_(urlexpire),
    url_(url),
    md5_(md5),
    urlTs_(urlTs),
    failTimes_(0),
    maxTimes_(3)
{

}

FileMessage::~FileMessage()
{
    
}

bool FileMessage::operator==(const FileMessage& other)
{
    return this->id_ == other.id_;
}

std::string FileMessage::getId()
{
    return id_;
}

std::string FileMessage::getMd5()
{
    return md5_;
}

bool FileMessage::equalOriginCached()
{   
    return file_ == cachedFile_;
}

std::string FileMessage::getUrl()
{
    return url_;
}

time_t FileMessage::getUrlGenTs()
{
    return urlTs_;
}

time_t FileMessage::getTimeStamp()
{
    return timestamp_;
}

File FileMessage::getFileHandler()
{
    return file_;
}

File FileMessage::getCacheHandler()
{
    return cachedFile_;
}

uint8_t FileMessage::getType()
{
    return type_;
}

void FileMessage::setUrl(const std::string url)
{
    url_ = url;
}

void FileMessage::setUrlExpire(uint32_t expire)
{
    expireSec_ = expire;
}

uint32_t FileMessage::getExpire()
{
    return expireSec_;
}

void FileMessage::setUrlGenTs(time_t ts)
{
    urlTs_ = ts;
}

void FileMessage::setType(uint8_t type)
{
    type_ = type;
}

bool FileMessage::needDeleteOrigin()
{
    return needDelete_;
}

void FileMessage::clearUrl()
{
    url_.clear();
    expireSec_ = 0;
    urlTs_ = 0;
    clearFailTimes();
}

void FileMessage::incrFailTimes()
{
    ++failTimes_;
}

void FileMessage::clearFailTimes()
{
    failTimes_ = 0;
}

void FileMessage::setFailTimes(uint8_t times)
{
    maxTimes_ = times;
}

bool FileMessage::needPersistent()
{
    return failTimes_ > maxTimes_;
}

bool FileMessage::isUrlExpired()
{
    time_t now = ::time(NULL);
    return (now - urlTs_) > expireSec_;
}

MessageQ::MessageQ(uint32_t maxlen):
    maxlen_(maxlen)
{

}

MessageQ::MessageQ():
    maxlen_(100)
{

}

MessageQ::~MessageQ()
{
    std::deque<FileMsgPtr> empty;
    queue_.swap(empty);
}

bool MessageQ::pushBack(const FileMsgPtr& fileMsgPtr)
{
    std::unique_lock<std::mutex> lock(mtx_);
    for (const auto& ptr: queue_) {
        if (ptr->getId() == fileMsgPtr->getId()) {
            HJ_INFO("[%s] exist, push message queue fail\n", ptr->getId().c_str());
            return false;
        }
    }
    
    if (queue_.size() == maxlen_) {
        std::sort(queue_.begin(), queue_.end(), [](FileMsgPtr msgA, FileMsgPtr msgB) {
	        return msgA->getTimeStamp() < msgB->getTimeStamp(); 
	    });
	    HJ_INFO("messages too much, discard oldest messages[%s]\n", (*queue_.begin())->getId().c_str());
        queue_.begin()->get()->getCacheHandler().remove();
        queue_.erase(queue_.begin());
    }

    HJ_INFO("push new file msg [%s] [%s]\n", fileMsgPtr->getId().c_str(), 
        fileMsgPtr->getFileHandler().getFullNameWithPath().c_str());
    queue_.emplace_back(fileMsgPtr);
}

FileMsgPtr MessageQ::getById(const std::string& id, bool remove)
{
    std::unique_lock<std::mutex> lock(mtx_);
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
        if ((*it)->getId() == id) {
            FileMsgPtr out = *it;
            if (remove) {
                queue_.erase(it);
            }
            return out;
        }
    }
    return nullptr;
}

FileMsgPtr MessageQ::getByFilePath(const std::string& filepath, bool remove)
{
    std::unique_lock<std::mutex> lock(mtx_);
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
        if ((*it)->getFileHandler().getFullNameWithPath() == filepath) {
            FileMsgPtr out = *it;
            if (remove) {
                queue_.erase(it);
            }
            return out;
        }
    }
    return nullptr;
}

FileMsgPtr MessageQ::popFront()
{
    std::unique_lock<std::mutex> lock(mtx_);
    if (queue_.empty()) {
        return nullptr;
    }

    auto out = queue_.front();
    queue_.pop_front();
    return out;
}

MessageQ& MessageQ::operator+=(MessageQ& other)
{
    std::unique_lock<std::mutex> lock(other.mtx_);
    for (const auto& msg: other) {
        this->pushBack(msg);
    }
    return *this;
}

size_t MessageQ::getSize()
{
    std::unique_lock<std::mutex> lock(mtx_);
    return queue_.size();
}

bool MessageQ::isEmpty()
{
    std::unique_lock<std::mutex> lock(mtx_);
    return queue_.empty();
}

void MessageQ::swapWithOther(std::deque<FileMsgPtr>& other)
{
    //Just for defense, should never oversize
    if (other.size() >= maxlen_) {
        std::sort(other.begin(), other.end(), [](FileMsgPtr msgA, FileMsgPtr msgB) {
	        return msgA->getTimeStamp() < msgB->getTimeStamp(); 
	    });

        while (other.size() >= maxlen_) {
            other.begin()->get()->getCacheHandler().remove();
            other.erase(other.begin());
        }
    }

    std::unique_lock<std::mutex> lock(mtx_);
    queue_.swap(other);
}

std::deque<FileMsgPtr>::const_iterator MessageQ::begin()
{
    return queue_.begin();
}

std::deque<FileMsgPtr>::const_iterator MessageQ::end()
{
    return queue_.end();
}

} //namespace collect_node_posttunnel