// @file function_factory.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)
#include "function_factory.h"

#include <dlfcn.h>
#include <unistd.h>

#include <fstream>
namespace hj_bf {
struct Function::FunctionConfig {
  std::string hj_interface_version;
  std::string name;
};

void Function::setHJInterfaceVersion(const std::string &interface_ver) {
  config_->hj_interface_version = interface_ver;
}

Function::Function(const rapidjson::Value &json_conf) {
  config_ = std::make_unique<FunctionConfig>();
  if (json_conf.HasMember("name")) {
    config_->name = json_conf["name"].GetString();
  } else {
    std::cerr << "function should have a name" << std::endl;
    throw std::runtime_error("function should have a name");
  }
}
Function::~Function() {}
const std::string &Function::getHJInterfaceVersion() { return config_->hj_interface_version; }
const std::string &Function::name() { return config_->name; }
/// NodeFactory
void FunctionFactory::registerFunction(const std::string &so_path) {
  void *handle = dlopen(so_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (handle == nullptr) {
    const char *err = dlerror();
    throw std::runtime_error("cant load library, so name: " + so_path + " errorcode:" + err);
  }
  void *sym = dlsym(handle, HJ_REGISTER_NAME_STR);
  if (sym == nullptr) {
    std::cerr << "cant find the name of function register,so=" << so_path << std::endl;
    return;
    //    throw std::runtime_error("cant find the name of function register");
  }
  ((void (*)(FunctionFactory &))sym)(*this);
}
std::unique_ptr<Function> FunctionFactory::instantiateFunction(const rapidjson::Value &json_conf) const {
  if (json_conf.HasMember("ID")) {
    std::string ID = json_conf["ID"].GetString();
    auto it = creaters_.find(ID);
    if (it == creaters_.end()) {
      std::cerr << ID << " not registered" << std::endl;
      throw std::runtime_error(ID + " not registered");
    }
    std::unique_ptr<Function> function = it->second(json_conf);
    return function;
  } else {
    std::cerr << "your config should have ID for creating function" << std::endl;
    throw std::runtime_error("your config should have ID for creating function");
  }
}
void FunctionFactory::insertCreater(const std::string &ID, FunctionCreater creater) {
  auto it = creaters_.find(ID);
  if (it != creaters_.end()) {
    throw std::runtime_error(ID + "already registered");
  }
  creaters_.insert(std::make_pair(ID, creater));
}
}  // namespace hj_bf
