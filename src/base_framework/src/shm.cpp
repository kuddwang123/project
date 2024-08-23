// @file function_factory.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-5-16)
#include "shm.h"

#include <mutex>

namespace hj_bf {
Shm* Shm::g_shm_ptr = nullptr;
Shm::Shm() {
  HJ_INFO("Shm in constructor");
  segment_ = std::make_shared<boost::interprocess::managed_shared_memory>(
      boost::interprocess::open_or_create, SHM_NAME, boost::interprocess::mapped_region::get_page_size() * SHM_SIZE);
  HJ_INFO("get_page_size:%ld", boost::interprocess::mapped_region::get_page_size());
}
void Shm::insertValue(const std::string& name, VariableImpl* ptr) {
  if (ptr != nullptr) {
    std::lock_guard<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
    variables_maps_[name] = ptr;
  } else {
    HJ_ERROR("insertValue error ptr is nullptr");
  }
}
void Shm::deleteValue(const std::string& name) {
  std::lock_guard<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  variables_maps_.erase(name);
}

Shm::~Shm() {
  HJ_ERROR("~Shm");
  boost::interprocess::shared_memory_object::remove(SHM_NAME);
  boost::interprocess::shared_memory_object::remove(SHM_NAME);
}
void Shm::createInstance() {
  static std::once_flag flag;
  //  static
  static Shm temp;
  std::call_once(flag, []() {
    g_shm_ptr = &temp;
    if (g_shm_ptr == nullptr) {
      std::cerr << "shm create error!" << std::endl;
      throw std::runtime_error("shm create error!");
    }
  });
  /* */
}

bool Shm::createVariable(const std::string& value_type, const std::string& name, VariableImpl* p_value) {
  {
    std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
    if (Shm::g_shm_ptr->variables_maps_.find(name) != Shm::g_shm_ptr->variables_maps_.end()) {
      lk.unlock();
      HJ_ERROR("createVariable fail ,have the same name %s", name.c_str());
      return false;
    }
  }
  Shm::g_shm_ptr->insertValue(name, p_value);
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("createVariable not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string value_name = SHM_TYPE_PREFIX + name;
  VariableImpl* p_type_value = Shm::g_shm_ptr->segment_->find_or_construct<VariableInstanceArray<char, SHM_TYPE_LEN>>(
      value_name.c_str())(value_type.c_str());
  Shm::g_shm_ptr->insertValue(value_name, p_type_value);
  return true;
}

bool Shm::insertVariable(const std::string& name, VariableImpl* ptr, bool insert) {
  if (insert) {
    Shm::g_shm_ptr->insertValue(name, ptr);
    return true;
  } else {
    std::string type_name = SHM_TYPE_PREFIX + name;
    Shm::g_shm_ptr->deleteValue(type_name);
    HJ_ERROR("setVariable have the type name :%s, but not have the value", type_name.c_str());
    return false;
  }
}

bool Shm::getVariable(const std::string& value_type, const std::string& name, bool& need_insert) {
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("getVariable not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string type_name = SHM_TYPE_PREFIX + name;
  std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  if (Shm::g_shm_ptr->variables_maps_.find(type_name) != Shm::g_shm_ptr->variables_maps_.end()) {
    std::string type_value =
        (reinterpret_cast<VariableInstanceArray<char, SHM_TYPE_LEN>*>(Shm::g_shm_ptr->variables_maps_[type_name]))
            ->getVariable();
    if (type_value == value_type) {
      return true;
    } else {
      lk.unlock();
      HJ_ERROR("getVariable error ,in type:%s, default type:%s", value_type.c_str(), type_value.c_str());
      return false;
    }
  } else {
    lk.unlock();
    std::pair<VariableInstanceArray<char, SHM_TYPE_LEN>*, std::size_t> get_shm_obj_name =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<char, SHM_TYPE_LEN>>(type_name.c_str());
    if (get_shm_obj_name.second != 0) {
      std::string temp_shm_obj_name_str = get_shm_obj_name.first->getVariable();
      if (temp_shm_obj_name_str == value_type) {
        Shm::g_shm_ptr->insertValue(type_name, get_shm_obj_name.first);
        need_insert = true;
        return false;
      } else {
        HJ_ERROR("getVariable error2 ,get type name:%s, default type:%s", value_type.c_str(),
                 get_shm_obj_name.first->getVariable());
        return false;
      }
    } else {
      HJ_ERROR("getVariable have not name :%s", name.c_str());
      return false;
    }
  }
}
bool Shm::setVariable(const std::string& value_type, const std::string& name, bool& need_insert, bool& need_create) {
  if (value_type.size() >= SHM_TYPE_LEN) {
    HJ_ERROR("setVariable not support too complex type  %s", value_type.c_str());
    return false;
  }
  std::string type_name = SHM_TYPE_PREFIX + name;
  std::unique_lock<std::mutex> lk(Shm::g_shm_ptr->variables_maps_mutex_);
  if (Shm::g_shm_ptr->variables_maps_.find(type_name) != Shm::g_shm_ptr->variables_maps_.end()) {
    std::string type_value =
        (reinterpret_cast<VariableInstanceArray<char, SHM_TYPE_LEN>*>(Shm::g_shm_ptr->variables_maps_[type_name]))
            ->getVariable();
    if (type_value == value_type) {
      return true;
    } else {
      lk.unlock();
      HJ_ERROR("setVariable error ,in type:%s, default type:%s", value_type.c_str(), type_value.c_str());
      return false;
    }
  } else {
    lk.unlock();
    std::pair<VariableInstanceArray<char, SHM_TYPE_LEN>*, std::size_t> get_shm_obj_name =
        Shm::g_shm_ptr->segment_->find<VariableInstanceArray<char, SHM_TYPE_LEN>>(type_name.c_str());
    if (get_shm_obj_name.second != 0) {
      std::string temp_shm_obj_name_str = get_shm_obj_name.first->getVariable();
      if (temp_shm_obj_name_str == value_type) {
        Shm::g_shm_ptr->insertValue(type_name, get_shm_obj_name.first);
        need_insert = true;
        return false;
      } else {
        HJ_ERROR("setVariable error2 ,in type:%s, default type:%s", value_type.c_str(),
                 get_shm_obj_name.first->getVariable());
        return false;
      }
    } else {
      need_create = true;
      return false;
    }
  }
}

struct MinosLock::MinosLockVariable {
  boost::interprocess::interprocess_mutex* p_mutex_;
  std::shared_ptr<boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>> p_lock_;
  std::string name_;
};

MinosLock::MinosLock(const std::string& name, bool lock_im) {
  variable_ptr_ = std::make_unique<MinosLockVariable>();
  variable_ptr_->name_ = LOCK_PREFIX + name;

  std::pair<boost::interprocess::interprocess_mutex*, std::size_t> get_shm_obj =
      Shm::g_shm_ptr->segment_->find<boost::interprocess::interprocess_mutex>(variable_ptr_->name_.c_str());
  if (get_shm_obj.second == 0) {
    variable_ptr_->p_mutex_ = Shm::g_shm_ptr->segment_->find_or_construct<boost::interprocess::interprocess_mutex>(
        variable_ptr_->name_.c_str())();
  } else {
    variable_ptr_->p_mutex_ = get_shm_obj.first;
  }
  if (variable_ptr_->p_mutex_ != nullptr) {
    variable_ptr_->p_lock_ =
        std::make_shared<boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>>(
            *(variable_ptr_->p_mutex_));
    if (lock_im == false) {
      variable_ptr_->p_lock_->unlock();
    }
  } else {
    std::cerr << "MinosLock create error!, name=" << variable_ptr_->name_ << std::endl;
    throw std::runtime_error("MinosLock create error!");
  }
}
// MinosLock::~MinosLock(){
//   p_lock_.reset();
// }

void MinosLock::lock() { variable_ptr_->p_lock_->lock(); }
void MinosLock::unlock() { variable_ptr_->p_lock_->unlock(); }
std::shared_ptr<boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>> MinosLock::getLock() {
  return variable_ptr_->p_lock_;
}
MinosLock::~MinosLock(){};
}  // namespace hj_bf
