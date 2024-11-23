// @file big_data.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-6-28)
#include "big_data.h"

#include <hj_interface/AppData.h>
#include <hj_interface/AppMsg.h>
#include <hj_interface/BigDataSave.h>
#include <hj_interface/BigdataUpload.h>
#include <hj_interface/FileUpload.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>

#include "hj_interface/AppOnlineType.h"
#include "log.h"
#include "node_factory.h"
#include "shm.h"
#include "shm_interface.h"
namespace big_data {

constexpr int kProcessTimeoutMinutes = 10;
constexpr int kBigDataMaxSize = 2 * 1000 * 1000;  // 2MB
constexpr int kMqttMsgSizeMax = 1024 * 20;        // mqtt msg size
constexpr char kBigdataImmediateRecordFile[] = "/userdata/hj/log/bigdata.json";
// constexpr char kBigdataImmediateRecordFile[] = "/home/wangqing/work_space/hj_workspace/bigdata.json";
constexpr char kBigdataConfigFilePath[] = "/data/hj/config/big_data_config.json";
constexpr char kBigdataSaveTopicName[] = "/big_data_cache";
constexpr char kBigdataReportAppTopicName[] = "/ReportApp";
constexpr char kBigdataUploadFileTopicName[] = "/upload/file";
constexpr char kBigdataKey[] = "bigdata";
constexpr char kIotOnlineStatus[] = "iot online status";
constexpr char kBigdataInitStatus[] = "bigdata init status";
constexpr char kAppOnlineTopicName[] = "/AppOnline";

struct ManagerVariables {
  std::deque<std::string> big_data_items;
  std::mutex big_data_items_mutex;
  std::mutex big_data_weakup_mutex;
  std::mutex big_data_immediate_file_mutex;
  std::condition_variable big_data_items_cond;
  int big_data_items_size{0};
  std::ofstream ofile;
  hj_bf::HJSubscriber big_data_msg_sub;
  hj_bf::HJSubscriber big_data_cmd_sub;
  hj_bf::HJSubscriber iot_status_sub;
  bool exit_flag{false};
  std::string big_data_immediate_record_file{kBigdataImmediateRecordFile};
};

/**
 * @brief 大数据管理类
 *
 */
class BigdataCollection {
 public:
  static BigdataCollection& GetInstance();
  ~BigdataCollection();
  friend void Init();

 private:
  BigdataCollection();
  BigdataCollection(const BigdataCollection&) = delete;
  BigdataCollection(BigdataCollection&&) = delete;
  BigdataCollection& operator=(const BigdataCollection&) = delete;
  BigdataCollection& operator=(BigdataCollection&&) = delete;
  void Init();
  void MsgProcesser();
  void SendImmediateItems();
  bool ReadImmediateItems();
  void IotOnlineCallback(const hj_interface::AppOnlineType::ConstPtr& msg);
  void SaveCallback(const hj_interface::BigDataSave::ConstPtr& msg);
  void CmdCallback(const hj_interface::BigdataUpload::ConstPtr& msg);
  std::unique_ptr<ManagerVariables> manager_ptr_;
};
BigdataCollection& g_instance_ptr = big_data::BigdataCollection::GetInstance();

static inline hj_bf::HJPublisher& PubMqttInstance() {
  static hj_bf::HJPublisher big_data_pub_mqtt =
      hj_bf::HJAdvertise<hj_interface::AppMsg>(kBigdataReportAppTopicName, 10);
  return big_data_pub_mqtt;
}

static inline hj_bf::HJPublisher& PubSaveInstance() {
  static hj_bf::HJPublisher big_data_pub_save =
      hj_bf::HJAdvertise<hj_interface::BigDataSave>(kBigdataSaveTopicName, 10);
  return big_data_pub_save;
}
static inline hj_bf::HJPublisher& PubS3FileInstance() {
  static hj_bf::HJPublisher big_data_pub_s3 =
      hj_bf::HJAdvertise<hj_interface::FileUpload>(kBigdataUploadFileTopicName, 10);
  return big_data_pub_s3;
}

// void InsertBigdata(std::string key, std::string value, BigdataCmd cmd) {
//   bool online_status = false;
//   hj_bf::getVariable("iot online status", online_status);
//   if ((BigdataCmd::kBigdataImmediate == cmd) && online_status) {
//     // send now
//     hj_interface::AppData temp_data;
//     temp_data.key = key;
//     temp_data.payload = value;
//     hj_interface::AppMsg temp_msg;
//     temp_msg.appdata.push_back(std::move(temp_data));
//     PubMqttInstance().publish(temp_data);
//   } else {
//     hj_interface::BigDataSave temp_data;
//     temp_data.key = key;
//     temp_data.payload = value;
//     temp_data.cmd = static_cast<uint8_t> = cmd;
//     PubSaveInstance().publish(BigDataSave);
//   }
// }
static inline bool IsImmediate(const uint8_t& cmd) { return (0 != (cmd & kBigdataImmediate)); }
static inline bool IsPack(const uint8_t& cmd) { return (0 != (cmd & kBigdataPack)); }
static inline bool IsDelete(const uint8_t& cmd) { return (0 != (cmd & kBigdataDelete)); }

void InsertBigdata(const std::string& value, const std::string& param, uint8_t cmd) {
  if (value.empty()) {
    return;
  }
  static bool online_status = false;
  hj_bf::getVariable(kIotOnlineStatus, online_status);
  //  HJ_INFO("InsertBigdata value:%s, cmd:%d, param:%s", value.c_str(), cmd, param.c_str());
  if (IsImmediate(cmd) && online_status) {
    hj_interface::AppData temp_data;
    temp_data.key = kBigdataKey;
    temp_data.payload = value;
    hj_interface::AppMsg temp_msg;
    // temp_msg.appdata.push_back(std::move(temp_data));
    temp_msg.appdata.push_back(temp_data);
    temp_msg.to = hj_interface::AppMsg::BIGDATA;
    PubMqttInstance().publish(temp_msg);
    // HJ_INFO("InsertBigdata pub immediate");
  } else {
    hj_interface::BigDataSave temp_data;
    temp_data.key = kBigdataKey;
    temp_data.payload = value;
    temp_data.cmd = cmd;
    PubSaveInstance().publish(temp_data);
    // HJ_INFO("InsertBigdata pub save");
  }
  if (IsPack(cmd)) {
    hj_interface::FileUpload temp_path;
    temp_path.filePath = param;
    temp_path.deleteOnSuccess = static_cast<uint8_t>(IsDelete(cmd));
    temp_path.type = hj_interface::FileUpload::BURYPOINT;
    PubS3FileInstance().publish(temp_path);
    HJ_INFO("InsertBigdata pub s3");
  }
}

void BigdataCollection::IotOnlineCallback(const hj_interface::AppOnlineType::ConstPtr& msg) {
  static bool iot_online_status = false;
  HJ_INFO("IotOnlineCallback pub type:%d", msg->type);
  if (hj_interface::AppOnlineType::OFFLINE == msg->type || hj_interface::AppOnlineType::BT == msg->type) {
    iot_online_status = false;
    hj_bf::setVariable(kIotOnlineStatus, iot_online_status);
  } else if (hj_interface::AppOnlineType::BT_IOT == msg->type || hj_interface::AppOnlineType::IOT == msg->type) {
    iot_online_status = true;
    bool get_iot_online_status = false;
    hj_bf::getVariable(kIotOnlineStatus, get_iot_online_status);
    hj_bf::setVariable(kIotOnlineStatus, iot_online_status);
    if (false == get_iot_online_status) {
      manager_ptr_->big_data_items_cond.notify_one();
    }
  }
}

void BigdataCollection::SaveCallback(const hj_interface::BigDataSave::ConstPtr& msg) {
  if (!manager_ptr_->ofile.is_open()) {
    manager_ptr_->ofile.open(std::string(manager_ptr_->big_data_immediate_record_file), std::ios::app);
    if (!manager_ptr_->ofile.is_open()) {
      HJ_ERROR("cant open bigdata record file ,name :%s", manager_ptr_->big_data_immediate_record_file.c_str());
      return;
    }
  }
  std::unique_lock<std::mutex> fk(manager_ptr_->big_data_immediate_file_mutex);
  if (manager_ptr_->big_data_items_size > kBigDataMaxSize) {
    HJ_ERROR("bidata.json size::%d,Exceeding the threshold", manager_ptr_->big_data_items_size);
    return;
  }
  manager_ptr_->big_data_items_size += msg->payload.size();
  manager_ptr_->ofile << msg->payload << ",\n" << std::endl;
  fk.unlock();

  std::unique_lock<std::mutex> lk(manager_ptr_->big_data_items_mutex);
  manager_ptr_->big_data_items.emplace_back(msg->payload);
  lk.unlock();
}

void BigdataCollection::CmdCallback(const hj_interface::BigdataUpload::ConstPtr& msg) {
  // HJ_INFO("CmdCallback in, payload:%s, upload_file:%s,type=%d", msg->payload.c_str(), msg->upload_file.c_str(),
  //         msg->type);
  big_data::InsertBigdata(msg->payload, msg->upload_file, (1 << (msg->type)));
}
void BigdataCollection::SendImmediateItems() {
  // HJ_INFO("SendImmediateItems");
  std::deque<std::string> big_data_items_temp;
  std::unique_lock<std::mutex> lk(manager_ptr_->big_data_items_mutex);
  std::unique_lock<std::mutex> fk(manager_ptr_->big_data_immediate_file_mutex);
  big_data_items_temp.swap(manager_ptr_->big_data_items);

  manager_ptr_->ofile.close();
  manager_ptr_->ofile.open(manager_ptr_->big_data_immediate_record_file, std::ios::trunc);
  //  HJ_ERROR("SendImmediateItems open file ,name :%s", manager_ptr_->big_data_immediate_record_file.c_str());
  manager_ptr_->big_data_items_size = 0;
  lk.unlock();
  fk.unlock();

  while (0 != big_data_items_temp.size()) {
    //    HJ_ERROR("SendImmediateItems open file ,size :%d", big_data_items_temp.size());
    std::string payload_temp;
    int payload_size = 0;
    if (1 == big_data_items_temp.size()) {
      payload_temp = big_data_items_temp.at(0);
      big_data_items_temp.erase(big_data_items_temp.begin());
    } else {
      payload_temp += '[';
      payload_size += 1;
      auto itr = big_data_items_temp.begin();
      for (; itr != big_data_items_temp.end(); itr++) {
        if (kMqttMsgSizeMax < payload_size) {
          break;
        }
        payload_temp.append(*itr);
        payload_temp.append(",\n");
        payload_size += (itr->size() + 2);
      }
      payload_temp.pop_back();
      payload_temp.pop_back();
      payload_temp += ']';
      if (kMqttMsgSizeMax < payload_size) {
        big_data_items_temp.erase(big_data_items_temp.begin(), itr);
      } else {
        big_data_items_temp.clear();
      }
    }

    hj_interface::AppData temp_data;
    temp_data.key = kBigdataKey;
    temp_data.payload = payload_temp;
    hj_interface::AppMsg temp_msg;
    temp_msg.appdata.push_back(std::move(temp_data));
    temp_msg.to = hj_interface::AppMsg::BIGDATA;
    PubMqttInstance().publish(temp_msg);
    HJ_INFO("SendImmediateItems json:%s", payload_temp.c_str());
  }
}

// void BigdataCollection::WriteFile(std::ofstream& file, std::string value){
//   file << value <<",\n";
// }
bool BigdataCollection::ReadImmediateItems() {
  // HJ_INFO("ReadImmediateItems");
  std::ifstream in(manager_ptr_->big_data_immediate_record_file, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    HJ_ERROR("open robot and cant open bigdata file ,name  :%s", manager_ptr_->big_data_immediate_record_file.c_str());
    return false;
  }
  std::unique_lock<std::mutex> fk(manager_ptr_->big_data_immediate_file_mutex);
  in.seekg(0, std::ios::end);
  size_t length = in.tellg();
  std::string data;
  // HJ_INFO("ReadImmediateItems1 length:%d", length);
  if (length > 0) {
    in.seekg(0, std::ios::beg);
    //    data.reserve(length + 10);
    data.resize(length + 10);
    data[0] += '[';
    in.read(const_cast<char*>(&data[1]), length);
    auto ptr = data.begin() + length;
    *(ptr - 2) = ']';
  }
  manager_ptr_->big_data_items_size = length;
  fk.unlock();
  in.close();

  HJ_INFO("ReadImmediateItems1 read file:%s\n size:%ld\n", data.c_str(), data.size());
  if (length > 0) {
    rapidjson::Document document;
    document.Parse(data.c_str());
    // rapidjson::ParseResult ok =
    //     document.ParseStream<rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag>(data);
    if (!document.HasParseError() && document.IsArray()) {
      std::unique_lock<std::mutex> lk(manager_ptr_->big_data_items_mutex);
      for (rapidjson::Value::ConstValueIterator itr = document.Begin(); itr != document.End(); ++itr) {
        // 遍历数组中的每个元素
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        itr->Accept(writer);
        manager_ptr_->big_data_items.emplace_back(buffer.GetString());
      }
      for (auto itr = manager_ptr_->big_data_items.begin(); itr != manager_ptr_->big_data_items.end(); itr++) {
        HJ_INFO("ReadImmediateItems items:%s", itr->c_str());
      }
    } else {
      HJ_ERROR("cant analysis bigdata string to json");
    }
  }
  return true;
}
void BigdataCollection::MsgProcesser() {
  const char* big_data_config_file_ptr = getenv("BIG_DATA_CONFIG_FILE");
  std::string new_big_data_config_file((big_data_config_file_ptr != nullptr) ? big_data_config_file_ptr
                                                                             : kBigdataConfigFilePath);

  std::ifstream in(new_big_data_config_file, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    HJ_ERROR(" cant open bigdata config file fail,name  :%s", new_big_data_config_file.c_str());
  } else {
    rapidjson::Document document;
    rapidjson::IStreamWrapper isw(in);
    document.ParseStream(isw);

    if (document.HasParseError()) {
      HJ_ERROR("cant analysis bigdata file name:%s", new_big_data_config_file.c_str());
    }
    if (document.IsObject()) {
      if (document.HasMember("save_file_path") && document["save_file_path"].IsString()) {
        manager_ptr_->big_data_immediate_record_file = document["save_file_path"].GetString();
      }
    }
  }

  HJ_INFO("MsgProcesser");
  bool get_iot_online_status = false;
  ReadImmediateItems();
  std::unique_lock<std::mutex> lk(manager_ptr_->big_data_items_mutex);
  lk.unlock();
  std::unique_lock<std::mutex> weakup_lk(manager_ptr_->big_data_weakup_mutex);
  while (true) {
    if (std::cv_status::timeout ==
        manager_ptr_->big_data_items_cond.wait_for(weakup_lk, std::chrono::minutes(kProcessTimeoutMinutes))) {
      // timeout
      // HJ_INFO("MsgProcesser timeout");
    }

    hj_bf::getVariable(kIotOnlineStatus, get_iot_online_status);
    lk.lock();
    int items_num = manager_ptr_->big_data_items.size();
    lk.unlock();
    if (true == get_iot_online_status && (0 < items_num)) {
      SendImmediateItems();
    }
    if (true == manager_ptr_->exit_flag) {
      HJ_IMPORTANT("bigdata process exit");
      break;
    }
  }
}

BigdataCollection& BigdataCollection::GetInstance() {
  static BigdataCollection instance;
  return instance;
}

BigdataCollection::BigdataCollection() {
  std::cerr << "BigdataCollection constructor" << std::endl;
  hj_bf::Shm::createInstance();
  std::cerr << "BigdataCollection here" << std::endl;

  bool iot_online_status = false;
  bool ret = hj_bf::getVariable(kIotOnlineStatus, iot_online_status);
  if (false == ret) {
    hj_bf::setVariable(kIotOnlineStatus, iot_online_status);
  }
}

BigdataCollection::~BigdataCollection() {
  std::cerr << "~BigdataCollection" << std::endl;
  if (manager_ptr_.get() != nullptr) {
    manager_ptr_->exit_flag = true;
    manager_ptr_->big_data_items_cond.notify_one();
  }
}
void Init() { g_instance_ptr.Init(); }

void BigdataCollection::Init() {
  bool init_status = false;
  bool ret = false;
  {
    hj_bf::MinosLock my_lock("bigdata init lock");
    ret = hj_bf::getVariable(kBigdataInitStatus, init_status);
    if ((false != ret) && (init_status == true)) {
      my_lock.unlock();
      std::cerr << "BigdataCollection::Init() already create." << std::endl;
      return;
    }
    init_status = true;
    hj_bf::setVariable(kBigdataInitStatus, init_status);
  }
  manager_ptr_ = std::make_unique<ManagerVariables>();
  std::thread process_thread(&BigdataCollection::MsgProcesser, this);
  process_thread.detach();
  manager_ptr_->big_data_msg_sub =
      hj_bf::HJSubscribe(big_data::kBigdataSaveTopicName, 10, &BigdataCollection::SaveCallback, &g_instance_ptr);
  manager_ptr_->iot_status_sub =
      hj_bf::HJSubscribe(kAppOnlineTopicName, 10, &BigdataCollection::IotOnlineCallback, &g_instance_ptr);
  manager_ptr_->big_data_cmd_sub =
      hj_bf::HJSubscribe(big_data::kBigdataCmdTopicName, 10, &BigdataCollection::CmdCallback, &g_instance_ptr);
}
}  // namespace big_data
