#include "handle_action.h"
#include "logic_dev/communication.h"
#include "boost/filesystem.hpp"
#include "log.h"

namespace collect_handle_action {

constexpr char kbigDataJsonPath[] = "/userdata/hj/log/bigdata.json";
constexpr char kSensorDataPath[] = "/userdata/hj/log/sensor_data";
constexpr char kMcuKeyLogPath[] = "/userdata/hj/log/mcukey";
constexpr char kNodeLogPath[] = "/tmp/logging";
constexpr char kDumpCollectNodePath[] = "/tmp/dump_collect_node";
constexpr char kDumpMiddlewareNodePath[] = "/tmp/dump_middleware_node";
constexpr char kDumpPlanningNodePath[] = "/tmp/dump_planning_node";
constexpr char kDumpSlamNodePath[] = "/tmp/dump_slam_node";
constexpr char kDumpUtilsNodePath[] = "/tmp/dump_utils_node";

const std::array<uint8_t, 2> kFuncions{
  kIotFunc,
  kLogFunc
};

void HandleAction::SysActionCallback(const hj_interface::SysAction &msg) {
  HJ_INFO("collect_node receive action:%d", msg.action);
  if (msg.from == hj_interface::SysAction::COMM_NODE_MIDDLEWARE &&
      (msg.action == hj_interface::SysAction::SYS_ACTION_RESTORE_FACTORY ||
       msg.action == hj_interface::SysAction::SYS_ACTION_RESET)) {
    function_count_.store(0);
    hj_interface::CollectBroadcast broadcast;
    broadcast.action = msg.action;
    pub_collect_action_.publish(msg);
  } else {
    hj_interface::SysAction response_action;
    response_action.from = hj_interface::SysAction::COMM_NODE_COLLECT;
    response_action.action = msg.action;
    response_action.res = 0;
    sys_action_response_.publish(response_action);
  }
}

void HandleAction::FuncResponeCallback(const hj_interface::CollectBroadcast &msg) {
  function_count_.fetch_add(1);
  uint8_t count = function_count_.load();

  // function 都处理完成，回复middleware
  if (count == kFuncions.size()) {
    hj_interface::SysAction response_action;
    response_action.from = hj_interface::SysAction::COMM_NODE_COLLECT;
    response_action.action = msg.action;
    response_action.res = 0;
    sys_action_response_.publish(response_action);
  }
}

void HandleAction::ResetLogHandler(const hj_interface::CollectBroadcast &msg) {
  // TODO(wuhao): delete the log file
  boost::filesystem::path path(kbigDataJsonPath);
  boost::filesystem::remove(path);
  path = boost::filesystem::path(kSensorDataPath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kMcuKeyLogPath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kNodeLogPath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kDumpCollectNodePath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kDumpMiddlewareNodePath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kDumpPlanningNodePath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kDumpSlamNodePath);
  boost::filesystem::remove_all(path);
  path = boost::filesystem::path(kDumpUtilsNodePath);
  boost::filesystem::remove_all(path);

  hj_interface::CollectBroadcast response_msg;
  response_msg.action = msg.action;
  response_msg.ack = 0;
  response_msg.function = function_type::kLogFunc;
  pub_func_response_.publish(response_msg);
}

HandleAction::HandleAction() {
  function_count_.store(0);
  sub_sys_action_ = hj_bf::HJSubscribe(kSysActionReq, 10, &HandleAction::SysActionCallback, this);
  sys_action_response_ = hj_bf::HJAdvertise<hj_interface::SysAction>(kSysActionResp, 10);
  sub_func_reponse_ = hj_bf::HJSubscribe("func/response/collect_node", 10, &HandleAction::FuncResponeCallback, this);
  pub_collect_action_ = hj_bf::HJAdvertise<hj_interface::CollectBroadcast>("collect_node/notify/func", 10);

  sub_collect_action_ = hj_bf::HJSubscribe("collect_node/notify/func", 10, &HandleAction::ResetLogHandler, this);
  pub_func_response_ = hj_bf::HJAdvertise<hj_interface::CollectBroadcast>("func/response/collect_node", 10);
}

HandleAction& HandleAction::GetInstance() {
  static HandleAction instance;
  return instance;
}
}  // namespace collect_handle_action
