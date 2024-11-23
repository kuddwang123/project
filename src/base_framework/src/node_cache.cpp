#include <mutex>
#include <condition_variable>
#include <iostream>
#include <thread>
#include "node_cache.h"
#include "log.h"


namespace hj_bf {
  std::mutex node_mutex_;
  std::mutex temp_mutex_;
  std::condition_variable cv_;
  DequeCache* g_cache = nullptr;

/**
 * 节点线程函数，用于srv send
*/
void HjSendSrv();

/**
 * set cache成员变量client_
*/
void HjSetClient(const hj_bf::HJClient& client);

void HjPushSrv(const hj_interface::HealthCheckCode& srv) {
  g_cache->push(srv);
}

void HjSendSrv() {
  g_cache->send_msg();
}

void HealthCheckInit() {
  DequeCache::createInstance(MAX_NUM);
  assert(g_cache);
  hj_bf::HJClient client = hj_bf::HJCreateClient<hj_interface::HealthCheckCode>(HJ_HEALTH_MONITOR);
  hj_bf::HjSetClient(client);
  auto state = std::thread(&hj_bf::HjSendSrv);  // 开线程
  state.detach();
}

void HjSetClient(const hj_bf::HJClient& client) {
  g_cache->setClient(client);
}

void HjSetService() {
  g_cache->setService();
}

void registerServerCallback(const srvCallBack& callback) {
  g_cache->callbacks_.push_back(callback);
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

void DequeCache::setService() {
  service_ = hj_bf::HJCreateServer(HJ_HEALTH_MONITOR, &DequeCache::server_callback, this);
}

bool DequeCache::server_callback(hj_interface::HealthCheckCodeRequest& req,
                       hj_interface::HealthCheckCodeResponse& res) {
  for (auto& callback : callbacks_) {
    callback(req, res);
  }
  res.result = 1;
  return true;
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
