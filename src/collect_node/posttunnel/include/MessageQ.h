#include <string>
#include <memory>
#include <deque>
#include "File.h"
#include <mutex>
namespace collect_node_posttunnel {
class FileMessage
{
public:
    explicit FileMessage(const File& file, uint8_t type,
                bool needDelete);

    explicit FileMessage(const std::string id,
        uint8_t type,
        bool needDelete, 
        const std::string& filepath,
        const std::string& filepathca,
        time_t ts,
        const std::string& md5
        );
    ~FileMessage();
    
    bool operator==(const FileMessage& other);
    std::string getId();
    bool equalOriginCached();
    std::string getUrl();
    time_t getTimeStamp();
    time_t getUrlGenTs();
    File getFileHandler();
    File getCacheHandler();
    std::string getMd5();
    uint32_t getExpire();
    uint64_t getLogId();
    uint8_t getType();
    void setUrl(const std::string url);
    void setUrlExpire(uint32_t expire);
    void setUrlGenTs(time_t ts);
    void setLogId(uint64_t logid);
    void clearUrl();
    void incrFailTimes();
    void clearFailTimes();
    void setFailTimes(uint8_t times);
    void setType(uint8_t type);
    bool needPersistent();
    bool isUrlExpired();
    bool needDeleteOrigin();
    static std::string cacheDir;

private:
    std::string id_;
    time_t timestamp_;
    uint8_t type_;
    bool needDelete_;
    File file_;
    File cachedFile_;
    uint32_t expireSec_;
    std::string url_;
    std::string md5_;
    time_t urlTs_;
    uint8_t failTimes_;
    uint8_t maxTimes_;
    uint64_t controlLogId_;
};

typedef std::shared_ptr<FileMessage> FileMsgPtr;
class MessageQ
{
public:
    explicit MessageQ(uint32_t maxlen);
    MessageQ();
    ~MessageQ();

    bool pushBack(const FileMsgPtr&);
    FileMsgPtr getById(const std::string& id, bool remove = false);
    FileMsgPtr getByFilePath(const std::string& filepath, bool remove = false);
    FileMsgPtr popFront();
    size_t getSize();
    bool isEmpty();
    void swapWithOther(std::deque<FileMsgPtr>& other);
    std::deque<FileMsgPtr>::const_iterator begin();
    std::deque<FileMsgPtr>::const_iterator end();

    MessageQ& operator+=(MessageQ& other);

private:
    uint32_t maxlen_;
    std::deque<FileMsgPtr> queue_;
    std::mutex mtx_;
};
} //namespace collect_node_posttunnel