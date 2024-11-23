#include "Persistence.h"
#include <fstream>
#include "log.h"
#include "hj_interface/AppMsg.h"
#include "Utils.h"

namespace collect_node_iot {
pthread_once_t Persistence::ponce_ = PTHREAD_ONCE_INIT;
Persistence* Persistence::instance_ = nullptr;
std::string Persistence::folderPrefix_ = "/userdata/.iotCache/";
std::vector<std::string> Persistence::interestKey_ = {"GetAlarmReport","OtaResultReport"};

void Persistence::init()
{
    instance_ = new Persistence();
    if (!boost::filesystem::is_directory(folderPrefix_)) {
        assert(boost::filesystem::create_directories(folderPrefix_));
    }
}

Persistence& Persistence::instance()
{
    pthread_once(&ponce_, &Persistence::init);
    return *instance_;
}

Persistence::Persistence()
{
    HJ_INFO("persist construct\n");
}

Persistence::~Persistence()
{
    HJ_INFO("persist construct\n");
}

void Persistence::initialize(const AppDataRouterPtr& ptr)
{
    appOnline_.store(false);
    refreshFlag_.store(false);
    isRun_.store(true);
    appRouterPtr_.reset(ptr.get());

    for (const auto& key:interestKey_) {
        data_[key] = std::deque<rapidjson::Document>();
    }

    std::thread([&]() {
        boost::filesystem::path dic(folderPrefix_);
        for (const auto& file: boost::filesystem::directory_iterator(dic)) {
            if (boost::filesystem::is_regular_file(file.status()) &&
                file.path().extension() == ".json") {
                auto key = file.path().stem().string();
                auto it = std::find_if(interestKey_.begin(), interestKey_.end(), [&key](const std::string& item) {
                    return key == item;
                });
                if (it == interestKey_.end()) {
                    boost::filesystem::remove(file);
                    HJ_INFO("remove extra file [%s]", file.path().string().c_str());
                    continue;
                }

                loadJsonFile2Map(file.path());
            }
        }
    }).detach();

    appOnlineSub_ = hj_bf::HJSubscribe("/AppOnline", 1, &Persistence::AppOnlineCb, this);

    ckFileSizeTmr_ = hj_bf::HJCreateTimer("persistenChkFile", 600 * 1000 * 1000, &Persistence::checkFileSizeTmrCb, this);

    refreshFileTh_ = std::thread(&Persistence::refreshFileThread, this);
    refreshFileTh_.detach();
}

void Persistence::shutdown()
{
    isRun_.store(false);
    refreshFlag_.store(false);
    
    refreshAllData();
    HJ_INFO("persistent shutdown complete\n");
}

void Persistence::loadJsonFile2Map(const boost::filesystem::path& filePath)
{
    std::ifstream file(filePath.string());

    if (!file.is_open()) {
        HJ_ERROR("Could not open file: %s\n", filePath.string().c_str());
        return;
    }

    std::string jsonContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    rapidjson::Document document;
    rapidjson::ParseResult parseResult = document.Parse(jsonContent.c_str());

    if (!parseResult || !document.IsObject()) {
        HJ_ERROR("Json parse fail: %s\n", filePath.string().c_str());
        return;
    }

    if (document.HasMember("data") && document["data"].IsArray()) {
        const rapidjson::Value& dataArray = document["data"];
        std::deque<rapidjson::Document> queue;
        for (rapidjson::SizeType i = 0; i < dataArray.Size(); ++i) {
            rapidjson::Document doc;
            doc.CopyFrom(dataArray[i], doc.GetAllocator());
            HJ_INFO("load [%s]\n", utils::documentToString(doc).c_str());
            queue.push_back(std::move(doc));
        }
        std::unique_lock<std::mutex> lc(mtx_);
        data_[filePath.stem().string()] = std::move(queue);
    }
}

void Persistence::reset()
{
    isRun_.store(false);
    refreshFlag_.store(false);

    std::lock_guard<std::mutex> lc(mtx_);
    for (auto it = data_.begin(); it != data_.end(); ++it) {
        it->second.clear();
        if (refreshFile(it->first, {})) {
            HJ_INFO("clear %s.json success\n", it->first.c_str());
        } else {
            HJ_INFO("clear %s.json fail\n", it->first.c_str());
        }
    }
}

bool Persistence::needPersistent(const std::string& key)
{
    auto it = std::find_if(interestKey_.begin(), interestKey_.end(), [&key](const std::string& item) {
        return key == item;
    });

    if (it == interestKey_.end()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lc(mtx_);
    return !data_[key].empty() || !appOnline_.load();
}

void Persistence::persistMsg(const std::string& key, const std::string& payload)
{
    rapidjson::Document doc;
    
    if (doc.Parse(payload.data()).HasParseError() || !doc.IsObject()) {
        HJ_ERROR("parse error:\n%s\n", payload.c_str());
        return;
    }
    
    if (!(doc.HasMember("ts") && doc["ts"].IsString())) {
        HJ_ERROR("has no ts [%s], add one\n", payload.c_str());
        struct timespec ts;
        ::clock_gettime(CLOCK_REALTIME, &ts);
        int64_t timestamp = static_cast<int64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
        std::string tsstr = std::to_string(timestamp);
        doc.AddMember("ts", rapidjson::Value(tsstr.data(), doc.GetAllocator()).Move(), doc.GetAllocator());
    }
    
    std::string tsstr = doc["ts"].GetString();
    int64_t tsms = std::stoll(tsstr);

    if (tsms <= 999999999999) {
        HJ_ERROR("invalid mill ts: %ld\n", tsms);
        return;
    }

    {
        std::lock_guard<std::mutex> lc(mtx_);
        auto& queue = data_[key];
        if (!queue.empty()) {
            const rapidjson::Value& front = queue.front();
            std::string fts = front["ts"].GetString();
            int64_t tdiff = tsms - std::stol(fts);
            if (tdiff < 0) {
                HJ_ERROR("time diff error: %ld %ld\n", tsms, std::stol(fts));
                return;
            }
            if (int(tdiff/1000) > 3600 * 24 * 28) {
                HJ_INFO("ts out of date: %ld\n", std::stol(fts));
                queue.pop_front();
            }
        }

        HJ_INFO("persist iot: %s\n", utils::documentToString(doc).c_str());
        queue.push_back(std::move(doc));
    }

    refreshFlag_.store(true);
}

void Persistence::AppOnlineCb(const hj_interface::AppOnlineType::ConstPtr& msg)
{
    if (!appOnline_.load() && (msg->type == hj_interface::AppOnlineType::IOT || 
            msg->type == hj_interface::AppOnlineType::BT_IOT)) {
        HJ_INFO("set persistence online true\n");
        appOnline_.store(true);
        std::thread([&](){reportCacheMsg();}).detach();
    } else if (msg->type == hj_interface::AppOnlineType::OFFLINE || 
            msg->type == hj_interface::AppOnlineType::BT) {
        HJ_INFO("set persistence online false\n");
        appOnline_.store(false);
    }
}

void Persistence::refreshFileThread()
{
    while (isRun_.load()) {
        if (refreshFlag_.load()) {
            refreshAllData();
            refreshFlag_.store(false);
        }
        if (!isRun_.load()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void Persistence::pubIotMsg(const std::string& key, const std::string& payload, int type)
{
    HJ_INFO("pub %d iot:%s  %s\n",type, key.c_str(), payload.c_str());
    hj_interface::AppMsg appmsg;
    hj_interface::AppData appdata;
    appdata.key = key;
    appdata.payload = payload;
    appmsg.appdata.emplace_back(appdata);
    appmsg.to = type;
    appRouterPtr_->doAppMsgReport(appmsg);
}

void Persistence::reportCacheMsg()
{
    {
        std::lock_guard <std::mutex> lc(mtx_);
        if (isDataEmptyWithoutLock()) {
            HJ_INFO("no cached data, return\n");
            return;
        }

        for (auto it = data_.begin(); it != data_.end(); ++it) {
            if (!it->second.empty()) {
                const rapidjson::Value& pload = it->second.back();
                pubIotMsg(it->first, utils::documentToString(pload), hj_interface::AppMsg::SHADOW);   
            }
        }

        int ckcnt = 0;
        for (auto it = data_.begin(); it != data_.end(); ++it) {
            bool interflag = false;
            if (it->second.empty()) {
                continue;
            }
            
            while (!it->second.empty()) {
                if (ckcnt >= 10) {
                    ckcnt = 0;
                    if (!appOnline_.load()) {
                        interflag = true;
                        break;
                    }
                }
                const std::string ploadstr = utils::documentToString(it->second.front());
                pubIotMsg(it->first, ploadstr, hj_interface::AppMsg::CLOUD);
                it->second.pop_front();
                ++ckcnt;
            }

            refreshFlag_.store(true);

            if (interflag) {
                HJ_INFO("iot offline, publish interrupt\n");
                break;
            }
        }
    }
}

bool Persistence::isDataEmptyWithoutLock()
{
    for (auto it = data_.begin(); it != data_.end(); ++it) {
        if (!it->second.empty()) {
            return false;
        }
    }
    return true;
}

void Persistence::refreshAllData()
{
    std::lock_guard<std::mutex> lc(mtx_);
    for (auto it = data_.begin(); it != data_.end(); ++it) {
        refreshFile(it->first, it->second);
    }
}

bool Persistence::refreshFile(const std::string& key, const std::deque<rapidjson::Document>& data)
{
    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Value dataArray(rapidjson::kArrayType);

    std::ofstream outfile(folderPrefix_+key +".json", std::ios::out | std::ios::trunc);
    if (!outfile.is_open()) {
        HJ_ERROR("open [%s.json] fail\n", key.c_str());
        return false;
    }

    HJ_INFO("open %s.json success\n", (folderPrefix_+key).c_str());
    
    for (auto& msg: data) {
        rapidjson::Document value;
        value.CopyFrom(msg, doc.GetAllocator());
        dataArray.PushBack(value, doc.GetAllocator());
    }

    doc.AddMember("data", dataArray, doc.GetAllocator());
    outfile << utils::documentToString(doc);
    outfile.close();
    return true;
}

uintmax_t Persistence::getFolderSize() {
    uintmax_t totalSize = 0;

    boost::filesystem::path dic(folderPrefix_);
    for (const auto& file: boost::filesystem::directory_iterator(dic)) {
        if (boost::filesystem::is_regular_file(file.status())) {
            totalSize += boost::filesystem::file_size(file);
        }  
    }

    return totalSize;
}

void Persistence::decreaseHalfMsg()
{
    for (auto it = data_.begin(); it != data_.end(); ++it) {
        int halfsize = it->second.size() / 2;
        for (int i = 0; i < halfsize; ++i) {
            it->second.pop_front();
        }
    }
}

void Persistence::checkFileSizeTmrCb(const hj_bf::HJTimerEvent&)
{
    std::lock_guard<std::mutex> lc(mtx_);
    auto foldersize = getFolderSize();
    if (foldersize >= 1024 * 1024 * 50) {
        HJ_INFO("trigger decrease msg: %lu\n", foldersize);
        decreaseHalfMsg();
        refreshFlag_.store(true);
    }
}

}// namespace collect_node_iot