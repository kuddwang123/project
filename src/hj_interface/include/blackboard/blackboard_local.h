#ifndef BLACKBOARD_LOCAL_H
#define BLACKBOARD_LOCAL_H

#include <deque>
#include "blackboard.h"
namespace BT {
constexpr int cache_blockboard_num = 256;
constexpr int blackboard_pollsize = sizeof(BT::blackboard_val) * cache_blockboard_num;
class BlackboardLocal : public BlackboardImpl {
 public:
  BlackboardLocal() { blackboard_poll_.emplace_back(); }

  virtual const SafeAny::Any *get(const std::string &key) const {
    auto it = storage_.find(key);
    if ((it == storage_.end()) || (it->second->second == false)) {
      return nullptr;
    }
    return &(it->second->first);
  }

  virtual BT::blackboard_val *getAgent(const std::string &key) {
    auto it = storage_.find(key);
    if (it == storage_.end()) {
      set(key, SafeAny::Any(nullptr));
      BT::blackboard_val *temp = storage_[key];
      temp->second = false;
      return temp;
    }
    return it->second;
  }

  virtual void set(const std::string &key, SafeAny::Any &&value) {
    auto itor = storage_.find(key);
    if (itor != storage_.end()) {
      itor->second->first = std::move(value);
      itor->second->second = true;
    } else {
      BT::blackboard_val temp = std::make_pair(std::move(value), true);
      int object_len = sizeof(temp);
      if ((blackboard_poll_index_ + object_len) > blackboard_pollsize) {
        blackboard_poll_.emplace_back();
        blackboard_poll_index_ = 0;
      }
      auto &back = blackboard_poll_.back();
      BT::blackboard_val *ptr = new (&back.at(blackboard_poll_index_)) BT::blackboard_val(std::move(temp));
      storage_[key] = ptr;
      blackboard_poll_index_ += object_len;
    }
  }

  virtual bool contains(const std::string &key) const {
    auto itor = storage_.find(key);
    if (itor != storage_.end()) {
      if (itor->second->second) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  virtual void erase(const std::string &key) {
    auto itor = storage_.find(key);
    if (itor != storage_.end()) {
      itor->second->second = false;
      //        itor->second->first = nullptr;
    }
  }

 private:
  std::deque<std::array<char, sizeof(BT::blackboard_val) * cache_blockboard_num>> blackboard_poll_;
  std::unordered_map<std::string, BT::blackboard_val *> storage_;
  int blackboard_poll_index_ = 0;
  static constexpr int blackboard_poll_size_ = sizeof(BT::blackboard_val) * cache_blockboard_num;
};
}  // namespace BT

#endif  // BLACKBOARD_LOCAL_H
