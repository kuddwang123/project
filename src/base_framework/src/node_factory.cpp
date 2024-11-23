// @file node_factory.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#include "node_factory.h"

#include <dirent.h>
#include <sys/epoll.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <csignal>

#include "function_factory.h"
#include "memory"
#include <fstream>
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include "node_cache.h"
#include "log.h"
namespace hj_bf {
ManagerNode* g_manager_node_ptr = nullptr;
static std::map<int, FdCallBack> g_fd_callbacks;
struct ManagerNode::ManagerParam {
  std::unordered_map<std::string, std::pair<std::string, std::map<uint32_t, std::shared_ptr<ros::Publisher>>>>
      publishers;
  std::unordered_map<std::string, std::map<uint32_t, std::shared_ptr<ros::Subscriber>>> subscribers;
  std::unordered_map<std::string, std::map<uint32_t, std::shared_ptr<ros::ServiceClient>>> clients;
  std::unordered_map<std::string, std::shared_ptr<ros::ServiceServer>> servers;
  std::unordered_map<std::string, std::shared_ptr<ros::Timer>> timers;
  std::unordered_map<std::string, std::shared_ptr<ros::SteadyTimer>> steady_timers;
  uint32_t publishers_index = 0;
  uint32_t subscribers_index = 0;
  uint32_t clients_index = 0;

  std::mutex publishers_mutex;
  std::mutex subscribers_mutex;
  std::mutex clients_mutex;
  std::mutex servers_mutex;
  std::mutex timers_mutex;
  std::mutex steady_timers_mutex;
};

ManagerNode::ManagerNode(const std::shared_ptr<struct NodeConfig>& in_config) : spinner_(in_config->node_threads) ,config_ptr_(in_config){
  node_handle_ptr_ = std::make_shared<ros::NodeHandle>();
  manager_ptr_ = std::make_unique<ManagerParam>();
}
HJPublisher ManagerNode::CreatePublisher(const std::string& topic, const std::string& msg_type_name,
                                         std::shared_ptr<ros::Publisher> publisher_ptr) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->publishers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->publishers.find(topic);
  if (it == g_manager_node_ptr->manager_ptr_->publishers.end()) {
    std::map<uint32_t, std::shared_ptr<ros::Publisher>> temp;
    temp[g_manager_node_ptr->manager_ptr_->publishers_index] = publisher_ptr;
    g_manager_node_ptr->manager_ptr_->publishers[topic] = std::make_pair(msg_type_name, temp);
  } else {
    if (it->second.first != (msg_type_name)) {
      std::cerr << "Registering publishers with the same name and type is not allowed!!!!!" << std::endl;
      throw std::runtime_error("Registering publishers with the same name and type is not allowed!!!!");
    } else {
      it->second.second[g_manager_node_ptr->manager_ptr_->publishers_index] = publisher_ptr;
    }
  }
  g_manager_node_ptr->manager_ptr_->publishers_index++;
  return HJPublisher(topic,
                     g_manager_node_ptr->manager_ptr_->publishers[topic]
                         .second[g_manager_node_ptr->manager_ptr_->publishers_index - 1],
                     g_manager_node_ptr->manager_ptr_->publishers_index - 1);
}

HJSubscriber ManagerNode::CreateSubscriber(const std::string& topic, std::shared_ptr<ros::Subscriber> subscriber_ptr) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->subscribers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->subscribers.find(topic);
  if (it == g_manager_node_ptr->manager_ptr_->subscribers.end()) {
    std::map<uint32_t, std::shared_ptr<ros::Subscriber>> temp;
    temp[g_manager_node_ptr->manager_ptr_->subscribers_index] = subscriber_ptr;
    g_manager_node_ptr->manager_ptr_->subscribers[topic] = temp;
  } else {
    (it->second)[g_manager_node_ptr->manager_ptr_->subscribers_index] = subscriber_ptr;
  }
  g_manager_node_ptr->manager_ptr_->subscribers_index++;
  return HJSubscriber(
      topic,
      g_manager_node_ptr->manager_ptr_->subscribers[topic][g_manager_node_ptr->manager_ptr_->subscribers_index - 1],
      g_manager_node_ptr->manager_ptr_->subscribers_index - 1);
}

HJClient ManagerNode::CreateClient(const std::string& topic, std::shared_ptr<ros::ServiceClient> client_ptr) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->clients_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->clients.find(topic);
  if (it == g_manager_node_ptr->manager_ptr_->clients.end()) {
    std::map<uint32_t, std::shared_ptr<ros::ServiceClient>> temp;
    temp[g_manager_node_ptr->manager_ptr_->clients_index] = client_ptr;
    g_manager_node_ptr->manager_ptr_->clients[topic] = temp;
  } else {
    (it->second)[g_manager_node_ptr->manager_ptr_->clients_index] = client_ptr;
  }
  g_manager_node_ptr->manager_ptr_->clients_index++;
  return HJClient(topic,
                  g_manager_node_ptr->manager_ptr_->clients[topic][g_manager_node_ptr->manager_ptr_->clients_index - 1],
                  g_manager_node_ptr->manager_ptr_->clients_index - 1);
}

HJServer ManagerNode::CreateServer(const std::string& service, std::shared_ptr<ros::ServiceServer> server_ptr) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->servers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->servers.find(service);
  if (it == g_manager_node_ptr->manager_ptr_->servers.end()) {
    g_manager_node_ptr->manager_ptr_->servers[service] = server_ptr;
    return HJServer(service, g_manager_node_ptr->manager_ptr_->servers[service]);
  }
  std::cerr << "Registering server with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering server with the same name is not allowed!");
}

static int getEpollFd() {
  static std::once_flag flag;
  static int efd = 0;
  std::call_once(flag, [&]() { efd = epoll_create(10); });
  return efd;
}
void registerFdCtrl(int fd, FdCallBack callback) {
  auto it = g_fd_callbacks.find(fd);
  struct epoll_event event;
  event.events = EPOLLIN;
  event.data.fd = fd;
  if (callback == nullptr) {
    if (it != g_fd_callbacks.end()) {
      epoll_ctl(getEpollFd(), EPOLL_CTL_DEL, fd, &event);
      g_fd_callbacks.erase(it);
    }
  } else {
    if (it == g_fd_callbacks.end()) {
      epoll_ctl(getEpollFd(), EPOLL_CTL_ADD, fd, &event);
    }
    g_fd_callbacks[fd] = callback;
  }
}

void readConfigure(const std::string& config_file_name, std::shared_ptr<struct NodeConfig> out_config) {
  std::ifstream config_file;
  config_file.open(config_file_name, std::ios::in | std::ios::binary);
  if (!config_file.is_open()) {
    std::cerr << "config file not exist!" << std::endl;
    throw std::runtime_error("config file not exist!");
  }
  std::string data;
  config_file.seekg(0, std::ios::end);
  size_t length = config_file.tellg();

  config_file.seekg(0, std::ios::beg);
  //    data.reserve(length + 10);
  data.resize(length + 10);
  config_file.read(const_cast<char*>(&data[0]), length);
  config_file.close();
  rapidjson::Document document;

  document.Parse<rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag>(data.c_str());
  if (!document.HasParseError() && document.IsObject()) {
    if (document.HasMember("log_path_config") && document["log_path_config"].IsString()) {
      out_config->log_config_path = document["log_path_config"].GetString();
    }

    if (document.HasMember("all_log_close") && document["all_log_close"].IsBool()) {
      out_config->all_log_close = document["all_log_close"].GetBool();
    }

    if (document.HasMember("node_log_close") && document["node_log_close"].IsBool()) {
      out_config->node_log_close = document["node_log_close"].GetBool();
    }

    if (document.HasMember("node_threads") && document["node_threads"].IsUint()) {
      out_config->node_threads = document["node_threads"].GetUint();
    }
    if (document.HasMember("node_name") && document["node_name"].IsString()) {
      out_config->node_name = document["node_name"].GetString();
    }

    if (document.HasMember("so_dir") && document["so_dir"].IsString()) {
      out_config->so_path = document["so_dir"].GetString();
    }
    if (document.HasMember("functions") && document["functions"].IsArray()) {//??
      rapidjson::StringBuffer buffer;
      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
      document["functions"].Accept(writer);  // 将值写入缓冲区
      out_config->value_str = buffer.GetString();
    }
  } else {
    std::cerr << "cant analysis " << config_file_name << " string to json" << std::endl;
    throw std::runtime_error("cant analysis config file");
  }
}
void getConfigure(const std::string& file, const std::string& so_path) {
  FILE* fp = fopen(file.c_str(), "rb");
  if (fp == nullptr) {
    std::cerr << "config file not exist!" << std::endl;
    throw std::runtime_error("config file not exist!");
  }
  fseek(fp, 0L, SEEK_END);
  auto length = ftell(fp);
  fseek(fp, 0L, SEEK_SET);
  char readBuffer[length + 16];
  rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
  rapidjson::Document document;
  rapidjson::ParseResult ok =
      document.ParseStream<rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag>(is);
  fclose(fp);
  if (ok == nullptr) {
    char errstr[2000];
    snprintf(errstr, sizeof(errstr), "JSON parse error: %s (%u)", rapidjson::GetParseError_En(ok.Code()),
             static_cast<unsigned int>(ok.Offset()));
    std::cerr << errstr << std::endl;
    throw std::runtime_error("get config err!");
  }

  hj_bf::FunctionFactory factory;
  if (document.IsObject()) {
    if (document.HasMember("so_dir") && document["so_dir"].IsString()) {
      std::string so_dir_string = document["so_dir"].GetString();
      DIR* so_dir = nullptr;
      struct dirent* so_name = nullptr;
      so_dir_string = so_path + "/" + so_dir_string;
      so_dir = opendir(so_dir_string.c_str());
      if (so_dir == nullptr) {
        std::cerr << "open so dir fail!" << std::endl;
        throw std::runtime_error("open so dir fail!");
      }
      while ((so_name = readdir(so_dir)) != nullptr) {
        int d_name_len = strlen(so_name->d_name);
        if (d_name_len > 6) {
          if (0 == strncmp(so_name->d_name, "lib", 3) && 0 == strcmp(so_name->d_name + d_name_len - 3, ".so")) {
            auto path = so_dir_string + so_name->d_name;
            HJ_INFO("minos2 get the so, name:%s",path.c_str());
            factory.registerFunction(path);
          }
        }
      }
      closedir(so_dir);
    }
    if (document.HasMember("functions") && document["functions"].IsArray()) {
      const rapidjson::Value& functions = document["functions"];
      for (rapidjson::SizeType i = 0; i < functions.Size(); ++i) {
        const rapidjson::Value& function = functions[i];
        std::unique_ptr<hj_bf::Function> temp_function = factory.instantiateFunction(function);
        HJ_INFO("minos2 success to instantiate a function ,name:%s",temp_function->name().c_str());
        g_manager_node_ptr->hj_functions_[temp_function->name()] = std::move(temp_function);
      }
    }
  }
}
void nodeInstance() {
  hj_bf::FunctionFactory factory;

  DIR* so_dir = nullptr;
  struct dirent* so_name = nullptr;
  std::string so_path_str =
      g_manager_node_ptr->config_ptr_->so_path + "/" + g_manager_node_ptr->config_ptr_->node_name + "/";
  so_dir = opendir(so_path_str.c_str());
  if (so_dir == nullptr) {
    std::cerr << "open so dir fail!" << std::endl;
    throw std::runtime_error("open so dir fail!");
  }
  while ((so_name = readdir(so_dir)) != nullptr) {
    int d_name_len = strlen(so_name->d_name);
    if (d_name_len > 6) {
      if (0 == strncmp(so_name->d_name, "lib", 3) && 0 == strcmp(so_name->d_name + d_name_len - 3, ".so")) {
        auto path = so_path_str + so_name->d_name;
        HJ_INFO("minos get the so, name:%s", path.c_str());
        factory.registerFunction(path);
      }
    }
  }
  closedir(so_dir);

  rapidjson::Document document;
  document.Parse<rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag>(
      g_manager_node_ptr->config_ptr_->value_str.c_str());
  for (rapidjson::SizeType i = 0; i < document.Size(); ++i) {
    const rapidjson::Value& function = document[i];
    std::unique_ptr<hj_bf::Function> temp_function = factory.instantiateFunction(function);
    HJ_INFO("minos success to instantiate a function ,name:%s", temp_function->name().c_str());
    g_manager_node_ptr->hj_functions_[temp_function->name()] = std::move(temp_function);
  }
}

void deletePublisher(const std::string& publisher_name, uint32_t index) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->publishers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->publishers.find(publisher_name);
  if (it != g_manager_node_ptr->manager_ptr_->publishers.end()) {
    auto it_sub = it->second.second.find(index);
    if (it_sub != it->second.second.end()) {
      it->second.second.erase(index);
      if (it->second.second.empty()) {
        g_manager_node_ptr->manager_ptr_->publishers.erase(publisher_name);
      }
    } else {
      g_manager_node_ptr->manager_ptr_->publishers.erase(publisher_name);
    }
  }
}

void deleteSubscriber(const std::string& subscriber_name, uint32_t index) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->subscribers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->subscribers.find(subscriber_name);
  if (it != g_manager_node_ptr->manager_ptr_->subscribers.end()) {
    auto it_sub = it->second.find(index);
    if (it_sub != it->second.end()) {
      it->second.erase(index);
      if (it->second.empty()) {
        g_manager_node_ptr->manager_ptr_->subscribers.erase(subscriber_name);
      }
    } else {
      g_manager_node_ptr->manager_ptr_->subscribers.erase(subscriber_name);
    }
  }
}
void deleteClient(const std::string& service_name, uint32_t index) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->clients_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->clients.find(service_name);
  if (it != g_manager_node_ptr->manager_ptr_->clients.end()) {
    auto it_sub = it->second.find(index);
    if (it_sub != it->second.end()) {
      it->second.erase(index);
      if (it->second.empty()) {
        g_manager_node_ptr->manager_ptr_->clients.erase(service_name);
      }
    } else {
      g_manager_node_ptr->manager_ptr_->clients.erase(service_name);
    }
  }
}
void deleteServer(const std::string& service_name) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->servers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->servers.find(service_name);
  if (it != g_manager_node_ptr->manager_ptr_->servers.end()) {
    g_manager_node_ptr->manager_ptr_->servers.erase(service_name);
  }
}

void deleteTimer(const std::string& timer_name) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->timers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->timers.find(timer_name);
  if (it != g_manager_node_ptr->manager_ptr_->timers.end()) {
    g_manager_node_ptr->manager_ptr_->timers.erase(timer_name);
  }
}

void deleteSteadyTimer(const std::string& timer_name) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->steady_timers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->steady_timers.find(timer_name);
  if (it != g_manager_node_ptr->manager_ptr_->steady_timers.end()) {
    g_manager_node_ptr->manager_ptr_->steady_timers.erase(timer_name);
  }
}

uint32_t getAllPublisher() {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->publishers_mutex);
  uint32_t ret = g_manager_node_ptr->manager_ptr_->publishers.size();
  return ret;
}
uint32_t getNumPublisher(const std::string topic) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->publishers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->publishers.find(topic);
  if (it != g_manager_node_ptr->manager_ptr_->publishers.end()) {
    return (it->second.second).size();
  } else {
    return 0;
  }
}
uint32_t getNumSubscriber(const std::string topic) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->subscribers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->subscribers.find(topic);
  if (it != g_manager_node_ptr->manager_ptr_->subscribers.end()) {
    return (it->second).size();
  } else {
    return 0;
  }
}

uint32_t getNumServer() {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->servers_mutex);
  uint32_t ret = g_manager_node_ptr->manager_ptr_->servers.size();
  return ret;
}

uint32_t getAllSubscriber() {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->subscribers_mutex);
  uint32_t ret = g_manager_node_ptr->manager_ptr_->subscribers.size();
  return ret;
}

uint32_t getNumClient(const std::string topic) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->clients_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->clients.find(topic);
  if (it != g_manager_node_ptr->manager_ptr_->clients.end()) {
    return (it->second).size();
  } else {
    return 0;
  }
}
uint32_t getAllClient() {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->clients_mutex);
  uint32_t ret = g_manager_node_ptr->manager_ptr_->clients.size();
  return ret;
}

uint32_t getNumTimer() {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->timers_mutex);
  uint32_t ret = g_manager_node_ptr->manager_ptr_->timers.size();
  return ret;
}
ManagerNode::~ManagerNode() {
  std::cerr << "ManagerNode exit!" << std::endl;
  g_manager_node_ptr->spinner_.stop();
  ros::shutdown();
}
void ManagerNode::createInstance(const std::shared_ptr<struct NodeConfig>& in_config) {
  static std::once_flag flag;
  //  static
  static ManagerNode temp(in_config);
  std::call_once(
      flag,
      [](uint32_t count) {
        g_manager_node_ptr = &temp;
        if (g_manager_node_ptr == nullptr) {
          std::cerr << "ManagerNode create error!" << std::endl;
          throw std::runtime_error("ManagerNode create error!");
        }
      },
      in_config->node_threads);
  /* */
}

ros::NodeHandle& getHandle() { return *(g_manager_node_ptr->node_handle_ptr_); }
void nodeInit(int argc, char** argv, const std::string& node_name, const std::shared_ptr<struct NodeConfig>& in_config, uint32_t ops) {
  ros::init(argc, argv, node_name, ops);
  ManagerNode::createInstance(in_config);
}

void nodeStart() { g_manager_node_ptr->spinner_.start(); }
void nodeSpin() {
  while (ros::ok()) {
    struct epoll_event events[2];
    int next_timeout_ms = 1000;
    int event_cnt = epoll_wait(getEpollFd(), events, 1, next_timeout_ms);
    for (int i = 0; i < event_cnt; ++i) {
      int fd = events[i].data.fd;
      if ((events[i].events & EPOLLIN) != 0) {
        auto callback = g_fd_callbacks[fd];
        if (callback == nullptr) {
          std::cerr << "get a fd trigger ,but no callback ,fd=" << fd << std::endl;
        } else {
          callback(fd);
        }
      }
    }
  }
  for (auto it = g_manager_node_ptr->manager_ptr_->timers.begin(); it != g_manager_node_ptr->manager_ptr_->timers.end();
       it++) {
    it->second->stop();
  }
}

uint32_t HJPublisher::getNumSubscribers() const { return publisher_ptr_->getNumSubscribers(); }
HJPublisher::~HJPublisher() {
  if (2 == publisher_ptr_.use_count()) {
    deletePublisher(name_, index_);
  }
}

HJSubscriber::~HJSubscriber() {
  if (2 == subscriber_ptr_.use_count()) {
    deleteSubscriber(name_, index_);
  }
}

HJClient::~HJClient() {
  if (2 == client_ptr_.use_count()) {
    std::cerr << "will come here" << std::endl;
    deleteClient(name_, index_);
  }
}

bool HJClient::isValid() const { return client_ptr_->isValid(); }
bool HJClient::exists() { return client_ptr_->exists(); }
bool HJClient::waitForExistence(const double us) {
  if (us < 0) {
    return client_ptr_->waitForExistence(ros::Duration(-1));
  }
  return client_ptr_->waitForExistence(ros::Duration(us / 1000000));
}

HJServer::~HJServer() {
  if (2 == server_ptr_.use_count()) {
    deleteServer(name_);
  }
}

HJTimer::~HJTimer() {
  if (2 == timer_ptr_.use_count()) {
    deleteTimer(name_);
  }
}
void HJTimer::start() { timer_ptr_->start(); }
void HJTimer::stop() { timer_ptr_->stop(); }
void HJTimer::setPeriod(const double us) { timer_ptr_->setPeriod(ros::Duration(us / 1000000)); }
bool HJTimer::hasPending() { return timer_ptr_->hasPending(); }
/**/
HJTimer HJCreateTimer(const std::string name, double us, const HJTimerCallback& callback, bool autostart) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->timers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->timers.find(name);
  if (it == g_manager_node_ptr->manager_ptr_->timers.end()) {
    g_manager_node_ptr->manager_ptr_->timers[name] = std::make_shared<ros::Timer>(
        g_manager_node_ptr->node_handle_ptr_->createTimer(ros::Duration(us / 1000000), callback, false, autostart));
    return HJTimer(us, g_manager_node_ptr->manager_ptr_->timers[name], name);
  }
  std::cerr << "Registering timer with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering timer with the same name is not allowed!");
}

HJSteadyTimer HJCreateSteadyTimer(const std::string name, double us, const HJSteadyTimerCallback& callback, bool oneshot, bool autostart) {
  std::lock_guard<std::mutex> lk(g_manager_node_ptr->manager_ptr_->steady_timers_mutex);
  auto it = g_manager_node_ptr->manager_ptr_->steady_timers.find(name);
  if (it == g_manager_node_ptr->manager_ptr_->steady_timers.end()) {
    g_manager_node_ptr->manager_ptr_->steady_timers[name] = std::make_shared<ros::SteadyTimer>(
        g_manager_node_ptr->node_handle_ptr_->createSteadyTimer(ros::WallDuration(us / 1000000), callback, oneshot, autostart));
    return HJSteadyTimer(us, g_manager_node_ptr->manager_ptr_->steady_timers[name], name);
  }
  std::cerr << "Registering steady_timer with the same name is not allowed!" << std::endl;
  throw std::runtime_error("Registering steady_timer with the same name is not allowed!");
}

HJSteadyTimer::~HJSteadyTimer() {
  if (2 == timer_ptr_.use_count()) {
    deleteSteadyTimer(name_);
  }
}
void HJSteadyTimer::start() { timer_ptr_->start(); }
void HJSteadyTimer::stop() { timer_ptr_->stop(); }
void HJSteadyTimer::setPeriod(const double us) { timer_ptr_->setPeriod(ros::WallDuration(us / 1000000)); }
bool HJSteadyTimer::hasPending() { return timer_ptr_->hasPending(); }

void HighResolutionTimer::start(double interval, std::function<void()> callback) {
  if (running_) {
    stop();
  }
  interval_ = interval;
  callback_ = callback;
  running_ = true;
  timer_thread_ = std::thread([this]() {
    while (running_) {
      auto startTime = std::chrono::high_resolution_clock::now();
      if (callback_) {
        callback_();
      }
      auto endTime = std::chrono::high_resolution_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime);
      double remainingTime = interval_ - elapsedTime.count();
      if (remainingTime > 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(remainingTime));
      }
    }
  });
  timer_thread_.detach();
}
void HighResolutionTimer::stop() {
  if (running_) {
    running_ = false;
    if (timer_thread_.joinable()) {
      timer_thread_.join();
    }
  }
}
HighResolutionTimer::~HighResolutionTimer() { stop(); };

bool ok() {
  return ros::ok();
}

std::unordered_map<int, std::string> g_params = {std::make_pair(SIGSEGV, "SIGSEGV"), std::make_pair(SIGABRT, "SIGABRT"),
                                                 std::make_pair(SIGBUS, "SIGBUS")};
struct sigaction act {};
struct sigaction segv_act {};
struct sigaction abrt_act {};
struct sigaction bus_act {};

void handler(int signo) {
  std::cerr << "CALLBACK: SIGNAL:" << signo << std::endl;
  if (g_params.find(signo) != g_params.end()) {
    std::cerr << boost::stacktrace::stacktrace() << std::endl;
  }
  if (signo == SIGSEGV && sigaction(signo, &segv_act, nullptr) == -1) {
    std::cerr << "sigaction SIGSEGV error:" << std::endl;
  }
  if (signo == SIGABRT && sigaction(signo, &abrt_act, nullptr) == -1) {
    std::cerr << "sigaction SIGABRT error" << std::endl;
  }
  if (signo == SIGBUS && sigaction(signo, &bus_act, nullptr) == -1) {
    std::cerr << "sigaction SIGBUS error" << std::endl;
  }
}

void registerSignal() {
  act.sa_flags = SA_NODEFER | SA_RESETHAND;
  act.sa_handler = &handler;
  sigfillset(&act.sa_mask);

  if (sigaction(SIGSEGV, &act, &segv_act) == -1) {
    std::exit(EXIT_FAILURE);
  }
  if (sigaction(SIGABRT, &act, &abrt_act) == -1) {
    std::exit(EXIT_FAILURE);
  }
  if (sigaction(SIGBUS, &act, &bus_act) == -1) {
    std::exit(EXIT_FAILURE);
  }
}
}  // namespace hj_bf
