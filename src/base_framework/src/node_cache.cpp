#include <mutex>
#include <condition_variable>
#include <iostream>
#include <thread>
#include "node_cache.h"
#include "hj_interface/HealthCheckCode.h"
#include "hj_interface/BigdataUpload.h"
#include "log.h"
#include "status_code.h"
#include <fstream>
#include "rapidjson/document.h"


namespace hj_bf {
  std::mutex node_mutex_;
  std::mutex temp_mutex_;
  std::condition_variable cv_;
  DequeCache* g_cache = nullptr;
  static hj_bf::HJServer service_;
  static hj_bf::HJPublisher sg_big_data_pub;

static std::string GetVersion() {
  static std::string fw_version = "";
  if (!fw_version.empty()) {
    return fw_version;
  }
  std::ifstream stream1("/etc/version");  // 输入流
  rapidjson::Document document;
  std::string jsonString((std::istreambuf_iterator<char>(stream1)),
                  std::istreambuf_iterator<char>());

  if (!document.Parse(jsonString.data()).HasParseError()) {
    if (document.HasMember("fw_ver") && document["fw_ver"].IsString()) {
      fw_version = document["fw_ver"].GetString();
    }
  }
  return fw_version;
}

void HjPushSrv(const hj_interface::HealthCheckCode& srv) {
  hj_bf::HjPushHealthCode(srv);
#ifdef BIGDATA_UPLOAD
  if (srv.request.code_val == status_code::COLLECT_NODE_MAX ||
      srv.request.code_val == status_code::SLAM_INIT_DONE ||
      srv.request.code_val == status_code::PLANING_NODE_MAX ||
      srv.request.code_val == status_code::MCU_SELF_CHECK_DONE) {
    return;
  }
  uint32_t error_code = srv.request.code_val;
  uint8_t status = srv.request.status;
  double timestamp = ros::Time::now().toSec();
  int64_t timestamp_ms = static_cast<int64_t>(timestamp * 1000);
  std::string fw_version = GetVersion();
  std::string str = R"({"event": "errorEvent", "errorCode":)" +
      std::to_string(error_code)+ R"(, "status": )" + std::to_string(status) +
      R"(, "time": )" +  std::to_string(timestamp_ms) +  R"(, "fwVersion": ")" + fw_version + R"("})";
  hj_interface::BigdataUpload msg;
  msg.payload = str;
  msg.type = hj_interface::BigdataUpload::kBigdataImmediate;
  sg_big_data_pub.publish(msg);
#endif
}

void HjSendSrv() {
  g_cache->send_msg();
}

void HjPushHealthCode(const hj_interface::HealthCheckCode& srv) {
  g_cache->push(srv);
}

void HealthCheckInit() {
  DequeCache::createInstance(MAX_NUM);
  assert(g_cache);
  hj_bf::HJClient client = hj_bf::HJCreateClient<hj_interface::HealthCheckCode>(HJ_HEALTH_MONITOR);
  hj_bf::HjSetClient(client);
  sg_big_data_pub = hj_bf::HJAdvertise<hj_interface::BigdataUpload>("/big_data_cmd", 10);
  auto state = std::thread(&hj_bf::HjSendSrv);  // 开线程
  state.detach();
}

void HjSetClient(const hj_bf::HJClient& client) {
  g_cache->setClient(client);
}

void registerServerCallback(const srvCallBack& callback) {
  service_ = hj_bf::HJCreateServer(HJ_HEALTH_MONITOR, callback);
}

void DequeCache::createInstance(uint32_t max_size) {
  static std::once_flag flag;
  //  static
  static DequeCache temp(max_size);
  std::call_once(
    flag,
    [](uint32_t size) {
      g_cache = &temp;
      if (g_cache == nullptr) {
        HJ_ERROR("DequeCache create error!");
        throw std::runtime_error("ManagerNode create error!");
      }
    },
    max_size);
}

void DequeCache::push(const hj_interface::HealthCheckCode& val) {
  std::unique_lock<std::mutex> lck(node_mutex_);
  if (static_cast<int>(cache_.size()) < max_size_) {
    cache_.emplace_back(val);
    cv_.notify_one();
  } else {
    HJ_WARN("error code out of range, max=%d,size=%ld", max_size_, cache_.size());
  }
}

bool DequeCache::get_front(hj_interface::HealthCheckCode* val) {
  std::unique_lock<std::mutex> lock(node_mutex_);
  if (!cache_.empty()) {
    *val = cache_.front();
    cache_.pop_front();
    return true;
  }
  return false;
}

void DequeCache::pop() {
  cache_.pop_front();
}

void DequeCache::setClient(const hj_bf::HJClient& client) {
  client_ = client;
}

void DequeCache::send_msg() {
  hj_interface::HealthCheckCode srv;
  bool send_status = true;  // 发送状态, 默认为true, 发送失败则置为false
  bool res = false;
  while (true) {
    std::unique_lock<std::mutex> lock(temp_mutex_);
    cv_.wait_for(lock, std::chrono::seconds(1));  // service未启动，导致消息发送失败，设置超时机制轮询发送错误码
    while (!cache_.empty() || !send_status) {
      if (send_status) {
        res = get_front(&srv);
      }
      if (res || !send_status) {
        if (client_.call(srv)) {
          srv = {};
          send_status = true;
        } else {
          if (send_status) {
            HJ_WARN("Failed to call service process check code %d", res);
          }
          send_status = false;
          break;
        }
      }
    }
  }
}
}  // namespace hj_bf
