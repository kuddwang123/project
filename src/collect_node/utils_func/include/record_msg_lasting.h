/**
 * @file record_msg_lasting.h
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-08-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_LASTING_H_
#define SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_LASTING_H_
#include <string>
#include <vector>
#include <thread>
#include "function_factory.h"
#include "node_factory.h"
#include "hjlog.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "hj_interface/TempHumidity.h"
#include "hj_interface/Turbidity.h"
#include "hj_interface/Bat.h"
#include "hj_interface/PumpMotorSpeed.h"
#include "hj_interface/FanMotorSpeed.h"
#include "hj_interface/SensorTemp.h"
#ifdef HJ_T1pro
#include "hj_interface/DustPlugDetection.h"
#endif

namespace collect_node_utils_func {  // your namespace
typedef struct {
  uint8_t size;
  std::string record_path;
} recordMsgLastingParam;

class RecordMsgLasting {
 public:
  RecordMsgLasting() = default;
  ~RecordMsgLasting();
  void createInstance(const recordMsgLastingParam& param);
 private:
  void run(const std::string& dir, uint8_t size);
  bool initizlize(const std::string& file, uint8_t size);
#ifdef HJ_T1pro
  void DustPlugCallback(const hj_interface::DustPlugDetection::ConstPtr&);
#endif
  void TurbidityCallback(const hj_interface::Turbidity::ConstPtr&);
  void TempHumidityCallback(const hj_interface::TempHumidity::ConstPtr&);
  void BatCallback(const hj_interface::Bat::ConstPtr&);
  void PumpMotorSpeedCallback(const hj_interface::PumpMotorSpeed::ConstPtr&);
  void FanMotorSpeedCallback(const hj_interface::FanMotorSpeed::ConstPtr&);
  void OutWaterCallback(const std_msgs::UInt8MultiArray::ConstPtr&);
  void SensorsTempCallback(const hj_interface::SensorTemp::ConstPtr&);
  void TurnMotorHallCallback(const std_msgs::UInt8::ConstPtr&);
 private:
  void* bat_logger_{nullptr};
  void* out_water_logger_{nullptr};
  void* tur_logger_{nullptr};
  void* temp_humidity_logger_{nullptr};
  void* pump_motor_speed_logger_{nullptr};
  void* fan_motor_speed_logger_{nullptr};
  void* sen_temp_logger_{nullptr};
  void* turn_motor_hall_logger_{nullptr};
#ifdef HJ_T1pro
  void* dust_plug_logger_{nullptr};
#endif
  hj_bf::HJSubscriber sub_tur_;
  hj_bf::HJSubscriber sub_temp_humidity_;
  hj_bf::HJSubscriber sub_bat_;
  hj_bf::HJSubscriber sub_pump_motor_speed_;
  hj_bf::HJSubscriber sub_fan_motor_speed_;
  hj_bf::HJSubscriber sub_out_water_;
  hj_bf::HJSubscriber sub_sen_temp_;
  hj_bf::HJSubscriber sub_turn_motor_hall_;
#ifdef HJ_T1pro
  hj_bf::HJSubscriber sub_dust_plug_;
#endif
};
}  // namespace collect_node_utils_func

#endif  // SRC_COLLECT_NODE_UTILS_FUNC_INCLUDE_RECORD_MSG_LASTING_H_
