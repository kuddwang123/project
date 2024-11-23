// Copyright 2022 Dreame Technology Co.Ltd. All rights reserved.
#ifndef AVA_NODE_ROUTE_SRC_BLACKBOARD_ROBIN_MAP_H_
#define AVA_NODE_ROUTE_SRC_BLACKBOARD_ROBIN_MAP_H_

#include "blackboard/blackboard.h"
#include "robin_hood.h"
#include <deque>
#if SIMULATE == 1
#include "route_types/utility/utility.h"
#endif  // SIMULATE == 1
// #include "robin_map.h"

namespace ancp {
class BlackboardRobinMap : public BT::BlackboardImpl {
 public:
  BlackboardRobinMap() = default;

  const SafeAny::Any* get(const std::string& key) const override {
#if SIMULATE == 1
        TidCheck("get", key);
#endif  // SIMULATE == 1
        auto it = storage_.find(key);
        if (it == storage_.end()) {
            return nullptr;
        }
        return &(it->second);
    }

    void set(const std::string& key, SafeAny::Any&& value) override {
#if SIMULATE == 1
        TidCheck("set", key);
#endif  // SIMULATE == 1
        storage_[key] = std::move(value);
    }

    bool contains(const std::string& key) const override {
#if SIMULATE == 1
        TidCheck("contains", key);
#endif  // SIMULATE == 1
        return storage_.find(key) != storage_.end();
    }

    void erase(const std::string& key) override {
#if SIMULATE == 1
        TidCheck("erase", key);
#endif  // SIMULATE == 1
        if (contains(key)) {
            storage_.erase(key);
        }
    }
#if SIMULATE == 1
    static void SetTid(pid_t tid) {
        main_tid_ = tid;
    }
#endif  // SIMULATE == 1

 private:
#if SIMULATE == 1
    void TidCheck(const std::string &operation,
                  const std::string &key) const {
        if (main_tid_ != 0) {
            pid_t tid = GetTid();
            if (tid != main_tid_) {
                throw std::runtime_error(
                            std::string("Operation of blackboard from "
                                        "2 different threads detected!")
                            + "\nmain thread         : " + std::to_string(main_tid_)
                            + "\noperate from thread : " + std::to_string(tid)
                            + "\noperation           : " + operation
                            + "\nkey                 : " + key);
            }
        }
    }
#endif  // SIMULATE == 1

    // std::unordered_map<std::string, SafeAny::Any> storage_;
    robin_hood::unordered_node_map<std::string, SafeAny::Any> storage_;
    // tsl::robin_map<std::string, SafeAny::Any> storage_;
#if SIMULATE == 1
    static pid_t main_tid_;  // 黑板所在的主线程的tid
#endif  // SIMULATE == 1
};

// minos new map
constexpr int  cache_blockboard_num  = 256;
constexpr int blackboard_pollsize = sizeof(BT::blackboard_val) * cache_blockboard_num;
class StaticBlackboardRobinMap : public BT::BlackboardImpl {
 public:
    StaticBlackboardRobinMap() {blackboard_poll_.emplace_back();}

    const SafeAny::Any* get(const std::string& key) const override {
      auto it = storage_.find(key);
      if ((it == storage_.end()) || (it->second->second == false)) {
        return nullptr;
      }
      return &(it->second->first);
    }

    BT::blackboard_val* getAgent(const std::string& key) override {
      auto it = storage_.find(key);
      if (it == storage_.end()) {
        set(key, SafeAny::Any(nullptr));
        BT::blackboard_val * temp = storage_[key];
        temp->second = false;
        return temp;
      }
      return it->second;
    }

    void set(const std::string& key, SafeAny::Any&& value) override {
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

    bool contains(const std::string& key) const override {
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

    void erase(const std::string& key) override {
      auto itor = storage_.find(key);
      if (itor != storage_.end()) {
        itor->second->second = false;
//        itor->second->first = nullptr;
      }
    }

 private:
  std::deque<std::array<char, sizeof(BT::blackboard_val) * cache_blockboard_num>> blackboard_poll_;
  robin_hood::unordered_node_map<std::string, BT::blackboard_val *> storage_;
  int blackboard_poll_index_ = 0;
  static constexpr int blackboard_poll_size_ = sizeof(BT::blackboard_val) * cache_blockboard_num;
};
}  // namespace ancp
#endif  // AVA_NODE_ROUTE_SRC_BLACKBOARD_ROBIN_MAP_H_
