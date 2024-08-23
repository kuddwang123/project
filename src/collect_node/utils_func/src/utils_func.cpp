#include "utils_func.h"
#include "log_upload.h"
#include "handle_action.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_utils_func::UtilsFunc>(FUNCTION_NAME);
}

namespace collect_node_utils_func {


UtilsFunc::~UtilsFunc() {}

UtilsFunc::UtilsFunc(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  recordMsgParam param;
  if (json_conf.HasMember("record_param") && json_conf["record_param"].IsObject()) {
    auto record_param = json_conf["record_param"].GetObject();
    if (record_param.HasMember("space_limit") && record_param["space_limit"].IsInt()) {
      param.space_limit = record_param["space_limit"].GetInt();
    }
    if (record_param.HasMember("machine_version") && record_param["machine_version"].IsString()) {
      param.machine_version = record_param["machine_version"].GetString();
    }
    if (record_param.HasMember("topics") && record_param["topics"].IsArray()) {
      for (auto &topic : record_param["topics"].GetArray()) {
        if (topic.IsString()) {
          if (strcmp(topic.GetString(), "all") == 0) {
            param.record_all_topics = true;
            break;
          }
          param.topics.emplace_back(topic.GetString());
        }
      }
    }

    if (record_param.HasMember("enable") && record_param["enable"].IsInt()) {
      int record_enable = record_param["enable"].GetInt();
      if (record_enable == 1) {
        if (record_param.HasMember("dir") && record_param["dir"].IsString()) {
          param.record_path = record_param["dir"].GetString();
        } else {
          param.record_path = "/userdata/hj/log/sensor_data";
        }
        record_msg_obj_.createInstance(param);
      }
    }
  }

  monitor_obj_.Init();
  log_upload::LogUpload::GetInstance();
  collect_handle_action::HandleAction::GetInstance();
}
}  // namespace collect_node_utils_func
