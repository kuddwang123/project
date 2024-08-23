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
#ifndef SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_H_
#define SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_H_
#include <string>
#include <vector>
#include <thread>
#include <unordered_map>
#include "function_factory.h"
#include "node_factory.h"
#include "std_msgs/UInt8.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Nav.h"
#include "hj_interface/Mag.h"
#include "hj_interface/Depth.h"
#include "hj_interface/SocImu.h"
#include "hj_interface/ElecMotorCur.h"
#include "hj_interface/TempHumidity.h"
#include "hj_interface/Turbidity.h"
#include "hj_interface/Bat.h"
#include "hj_interface/PumpMotorSpeed.h"
#include "hj_interface/FanMotorSpeed.h"
#include "hj_interface/TripleUltra.h"
#include "hj_interface/DownRay.h"
#ifdef HJ_T1pro
#include "hj_interface/LeftTof.h"
#include "hj_interface/DustPlugDetection.h"
#else
#include "hj_interface/LeftBack.h"
#include "hj_interface/LeftFront.h"
#include "hj_interface/DownLeft.h"
#include "hj_interface/DownRight.h"
#endif
#include "hj_interface/Atime.h"


#define FILE_POLLING_WRITES_ENABLED  (0)  // 0: 关闭文件轮询写入 1: 开启文件轮询写入

namespace collect_node_utils_func {  // your namespace
typedef struct {
  bool record_all_topics;
  uint32_t space_limit;
  std::string machine_version;
  std::vector<std::string> topics;
  std::string record_path;
} recordMsgParam;

class RecordMsg {
 public:
  RecordMsg() = default;
  ~RecordMsg();
  void createInstance(const recordMsgParam& param);
 private:
  bool initizlize();
  bool CheckDisk();
  void ScheduledCheckDisk();
  float GetDirectorySize(const char *dir);
  void CloseFiles();
  void MotorChatterCallback(const hj_interface::Encoder::ConstPtr&);
  // void UlsChatterCallback(const hj_interface::Ultra::ConstPtr&);
  void TripleUltraCallback(const hj_interface::TripleUltra::ConstPtr&);
  void ImuChatterCallback(const hj_interface::Imu::ConstPtr&);
  void SocImuChatterCallback(const hj_interface::SocImu::ConstPtr&);
  void FallCallback(const hj_interface::DownRay::ConstPtr&);
#ifdef HJ_T1pro
  void TofChatterCallback(const hj_interface::LeftTof::ConstPtr&);
  void DustPlugCallback(const hj_interface::DustPlugDetection::ConstPtr&);
#else
  void LeftBackCallback(const hj_interface::LeftBack::ConstPtr&);
  void LeftFrontCallback(const hj_interface::LeftFront::ConstPtr&);
  void DownLeftCallback(const hj_interface::DownLeft::ConstPtr&);
  void DownRightCallback(const hj_interface::DownRight::ConstPtr&);
#endif
  void MagChatterCallback(const hj_interface::Mag::ConstPtr&);
  void MotorSetChatterCallback(const hj_interface::Nav::ConstPtr&);
  void PressureChatterCallback(const hj_interface::Depth::ConstPtr&);
  void MotorCurCallback(const hj_interface::ElecMotorCur::ConstPtr&);
  void TurbidityCallback(const hj_interface::Turbidity::ConstPtr&);
  void TempHumidityCallback(const hj_interface::TempHumidity::ConstPtr&);
  void BatCallback(const hj_interface::Bat::ConstPtr&);
  void PumpMotorSpeedCallback(const hj_interface::PumpMotorSpeed::ConstPtr&);
  void FanMotorSpeedCallback(const hj_interface::FanMotorSpeed::ConstPtr&);
  void OutWaterCallback(const std_msgs::UInt8::ConstPtr&);
  void WriteMotor(const hj_interface::Atime::ConstPtr&);
  void WriteImu(const hj_interface::Atime::ConstPtr&);
 private:
  bool writing_enabled_{true};  // true: 写入normalfile  false: 写入bakfile
  int space_limit_{30};  // 磁盘空间最小要求，MB
  std::string machine_version_{"P3.5"};
  std::unordered_map<std::string, bool> topics_;
  bool record_all_topics_{false};
  int fd_motor_{-1};
  int fd_imu_{-1};
  int fd_motor_time_{-1};
  int fd_imu_time_{-1};
  int fd_soc_imu_{-1};
  int fd_triple_ultra_{-1};
  int fd_mag_{-1};
  int fd_nav_{-1};
  int fd_pressure_{-1};
  int fd_motor_cur_{-1};
  int fd_tur_{-1};  // turbidity
  int fd_temp_humidity_{-1};
  int fd_bat_{-1};
  int fd_pump_motor_speed_{-1};
  int fd_fan_motor_speed_{-1};
  int fd_out_water_{-1};
int fd_down_ray_{-1};
#ifdef HJ_T1pro
  int fd_tof_{-1};
  int fd_dust_plug_{-1};
#else
  int fd_left_back_{-1};
  int fd_left_front_{-1};
  int fd_down_left_{-1};
  int fd_down_right_{-1};
#endif
  std::string log_prefix_;
  std::mutex mutex_;
  std::mutex mutex_space_;
  bool space_enabled_{true};

  hj_bf::HJSubscriber sub_motor_;
  hj_bf::HJSubscriber sub_imu_;
  // hj_bf::HJSubscriber sub_tof_;
  // hj_bf::HJSubscriber sub_uls_;
  hj_bf::HJSubscriber sub_mag_;
  hj_bf::HJSubscriber sub_motor_set_;
  hj_bf::HJSubscriber sub_pressure_;
  hj_bf::HJSubscriber sub_soc_imu_;
  hj_bf::HJSubscriber sub_motor_cur_;
  // hj_bf::HJSubscriber sub_fall_;
  hj_bf::HJSubscriber sub_tur_;
  hj_bf::HJSubscriber sub_temp_humidity_;
  hj_bf::HJSubscriber sub_bat_;
  hj_bf::HJSubscriber sub_pump_motor_speed_;
  hj_bf::HJSubscriber sub_fan_motor_speed_;
  hj_bf::HJSubscriber sub_out_water_;
  hj_bf::HJSubscriber sub_triple_ultra_;
  hj_bf::HJSubscriber sub_down_ray_;
#ifdef HJ_T1pro
  hj_bf::HJSubscriber sub_tof_;
  hj_bf::HJSubscriber sub_dust_plug_;
#else
  hj_bf::HJSubscriber sub_left_back_;
  hj_bf::HJSubscriber sub_left_front_;
  hj_bf::HJSubscriber sub_down_left_;
  hj_bf::HJSubscriber sub_down_right_;
#endif
  hj_bf::HJSubscriber imu_time_sub_;
  hj_bf::HJSubscriber motor_tmie_sub_;
};
}  // namespace collect_node_utils_func

#endif  // SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_H_
