#include "utils_func.h"
#include "log_upload.h"
#include "handle_action.h"
#include "process_cmd.h"
#include <std_msgs/Int32.h>
#include <fstream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_utils_func::UtilsFunc>(FUNCTION_NAME);
}

namespace collect_node_utils_func {
constexpr char kDevInfoFile[] = "/tmp/devInfo.json";

UtilsFunc::~UtilsFunc() {}

UtilsFunc::UtilsFunc(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  recordMsgParam param = {0};
  if (json_conf.HasMember("record_param_alg") && json_conf["record_param_alg"].IsObject()) {
    auto record_param = json_conf["record_param_alg"].GetObject();
    if (record_param.HasMember("space_limit") && record_param["space_limit"].IsInt()) {
      param.space_limit = record_param["space_limit"].GetInt();
    }
    if (record_param.HasMember("machine_version") && record_param["machine_version"].IsString()) {
      param.machine_version = record_param["machine_version"].GetString();
    }
    if (record_param.HasMember("task_enable") && record_param["task_enable"].IsInt()) {
      param.task_enable = record_param["task_enable"].GetInt();
    }

    if (record_param.HasMember("enable") && record_param["enable"].IsInt()) {
      int record_enable = record_param["enable"].GetInt();
      if (record_enable == 1) {
        if (record_param.HasMember("dir") && record_param["dir"].IsString()) {
          param.record_path = record_param["dir"].GetString();
        } else {
          param.record_path = "/userdata/hj/log/sensor_data_alg";
        }
        record_msg_obj_.createInstance(param);
      }
    }
  }
  recordMsgLastingParam lasting_param = {0};
  if (json_conf.HasMember("record_param_other") && json_conf["record_param_other"].IsObject()) {
    auto record_param_other = json_conf["record_param_other"].GetObject();
    if (record_param_other.HasMember("space_limit") && record_param_other["space_limit"].IsInt()) {
      lasting_param.size = record_param_other["space_limit"].GetInt();
    }
    if (record_param_other.HasMember("enable") && record_param_other["enable"].IsInt()) {
      int record_enable = record_param_other["enable"].GetInt();
      if (record_enable == 1) {
        if (record_param_other.HasMember("dir") && record_param_other["dir"].IsString()) {
          lasting_param.record_path = record_param_other["dir"].GetString();
        } else {
          lasting_param.record_path = "/userdata/hj/log/sensor_data_other";
        }
        record_msg_lasting_obj_.createInstance(lasting_param);
      }
    }
  }
  InitProcess();
  monitor_obj_.Init();
  log_upload::LogUpload::GetInstance();
  collect_handle_action::HandleAction::GetInstance();
  process_cmd::ProcessCmd::GetInstance();
}

void UtilsFunc::InitProcess() {
  HJ_ERROR(" come  InitProcess");
  w2_switch_pub_ = hj_bf::HJAdvertise<std_msgs::Int32>("/w2/bind", 1, true);
  std::string mode;
  std::ifstream in(kDevInfoFile, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    HJ_ERROR(" cant open,name  :%s", kDevInfoFile);
    return;
  } else {
    rapidjson::Document document;
    rapidjson::IStreamWrapper isw(in);
    document.ParseStream(isw);

    if (document.HasParseError()) {
      HJ_ERROR("cant analysis  file name:%s", kDevInfoFile);
      return;
    }
    if (document.IsObject()) {
      if (document.HasMember("mode") && document["mode"].IsString()) {
        mode = document["mode"].GetString();
      }
    } else {
      HJ_ERROR("file:%s, have no mode", kDevInfoFile);
    }
  }
  if (mode == "fac") {
    std_msgs::Int32 msg;
    msg.data = 3;
    HJ_INFO("trigger fac");
    w2_switch_pub_.publish(msg);

  } else if (mode == "normal") {
    HJ_INFO("mode is normal");
  }
}
}  // namespace collect_node_utils_func
