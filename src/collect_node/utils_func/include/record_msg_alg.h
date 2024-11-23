/**
 * @file record_msg.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief record sensor data to file
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_ALG_H_
#define SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_ALG_H_
#include <string>
#include <thread>
#include <mutex>
#include "function_factory.h"
#include "node_factory.h"
#include "hj_zip.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Nav.h"
#include "hj_interface/Mag.h"
#include "hj_interface/Depth.h"
#include "hj_interface/TripleUltra.h"
#include "hj_interface/DownRay.h"
#include "hj_interface/LeftBack.h"
#include "hj_interface/LeftFront.h"
#ifdef HJ_T1pro
#include "hj_interface/LeftTof.h"
#else
#include "hj_interface/DownLeft.h"
#include "hj_interface/DownRight.h"
#endif
#include "hj_interface/Atime.h"
#include "hj_interface/SensorDataRecord.h"


namespace collect_node_utils_func {  // your namespace
typedef struct {
  uint8_t task_enable;
  uint32_t space_limit;
  std::string machine_version;
  std::string record_path;
} recordMsgParam;

class RecordMsg {
 public:
  RecordMsg() = default;
  ~RecordMsg();
  void createInstance(const recordMsgParam& param);
 private:
  void Init(uint8_t task_enable);
  void SubTopic();
  bool OpenLogFile(const std::string& log_prefix);
  void StartRecording(const std::string& path);
  void FinishTask();
  void ScheduledCheckDisk();
  uint64_t GetDirectorySize(const char *dir);
  void CloseFiles();
  void MotorChatterCallback(const hj_interface::Encoder::ConstPtr&);
  void TripleUltraCallback(const hj_interface::TripleUltra::ConstPtr&);
  void ImuChatterCallback(const hj_interface::Imu::ConstPtr&);
  void LeftBackCallback(const hj_interface::LeftBack::ConstPtr&);
  void LeftFrontCallback(const hj_interface::LeftFront::ConstPtr&);
#ifdef HJ_T1pro
  void TofChatterCallback(const hj_interface::LeftTof::ConstPtr&);
  void FallCallback(const hj_interface::DownRay::ConstPtr&);
#else
  void DownLeftCallback(const hj_interface::DownLeft::ConstPtr&);
  void DownRightCallback(const hj_interface::DownRight::ConstPtr&);
#endif
  void MagChatterCallback(const hj_interface::Mag::ConstPtr&);
  void MotorSetChatterCallback(const hj_interface::Nav::ConstPtr&);
  void PressureChatterCallback(const hj_interface::Depth::ConstPtr&);
  void WriteMotor(const hj_interface::Atime::ConstPtr&);
  void WriteImu(const hj_interface::Atime::ConstPtr&);
  void DealTaskCallback(const hj_interface::SensorDataRecord::ConstPtr&);
  void UploadCallback(const std_msgs::Bool::ConstPtr&);
 private:
  std::atomic<bool> writing_enabled_{false};  // true: 写入  false: 不能写入
  std::atomic<bool> space_enabled_{true};  // true: 空间足够  false: 空间不足
  std::string cur_log_path_{"/userdata/hj/log/sensor_data_alg"};
  int space_limit_{30};  // 磁盘空间最小要求，MB
  int fd_motor_{-1};
  int fd_imu_{-1};
  int fd_motor_time_{-1};
  int fd_imu_time_{-1};
  int fd_triple_ultra_{-1};
  int fd_mag_{-1};
  int fd_nav_{-1};
  int fd_pressure_{-1};
  int fd_left_back_{-1};
  int fd_left_front_{-1};
#ifdef HJ_T1pro
  int fd_down_ray_{-1};
  int fd_tof_{-1};
#else
  int fd_down_left_{-1};
  int fd_down_right_{-1};
#endif
  std::string log_prefix_;
  std::mutex mtx_;

  hj_bf::HJSubscriber sub_motor_;
  hj_bf::HJSubscriber sub_imu_;
  hj_bf::HJSubscriber sub_mag_;
  hj_bf::HJSubscriber sub_motor_set_;
  hj_bf::HJSubscriber sub_pressure_;
  hj_bf::HJSubscriber sub_triple_ultra_;
  hj_bf::HJSubscriber sub_left_back_;
  hj_bf::HJSubscriber sub_left_front_;
#ifdef HJ_T1pro
  hj_bf::HJSubscriber sub_down_ray_;
  hj_bf::HJSubscriber sub_tof_;
#else
  hj_bf::HJSubscriber sub_down_left_;
  hj_bf::HJSubscriber sub_down_right_;
#endif
  hj_bf::HJSubscriber imu_time_sub_;
  hj_bf::HJSubscriber motor_tmie_sub_;

  hj_bf::HJSubscriber task_sub_;  // task sub
  hj_bf::HJSubscriber upload_sub_;  // upload sensor data sub
  hj_bf::HJPublisher pub_upload_;

};
}  // namespace collect_node_utils_func

#endif  // SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_ALG_H_
