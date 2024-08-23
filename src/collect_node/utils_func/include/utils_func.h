/**
 * @file utils_func.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-07-22
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_UTILS_FUNC_H_
#define SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_UTILS_FUNC_H_

#include <string>
#include <vector>
#include "function_factory.h"
#include "node_factory.h"
#include "monitor.h"
#include "record_msg.h"
#include "log.h"


namespace collect_node_utils_func {

class UtilsFunc : public hj_bf::Function {
 public:
  explicit UtilsFunc(const rapidjson::Value &json_conf);
  ~UtilsFunc();
 private:
  Monitor monitor_obj_;
  RecordMsg record_msg_obj_;
};

}  // namespace collect_node_utils_func

#endif  // SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_UTILS_FUNC_H_
