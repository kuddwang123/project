/**
 * @file imu.cpp
 * @author hao wu (clayderman@yardbot.net)
 * @brief 
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright 2023 HJ Technology Co.Ltd. All rights reserved
 * 
 */
#include "imu.h"
#include "log.h"
#include "node_cache.h"


HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_imu::Imu>(FUNCTION_NAME);
}

namespace collect_node_imu {

bool Imu::Iim42652Init() {
  uint8_t id = 0, data = 0;
  bool ret = false;

  i2c_.I2cReadBytes(SLAVE_ADDR, WHOAMI, &id, 1);
  if (DEVICE_ID != id) {
    HJ_ERROR("file:%s, FUNC:%s, line:%d, chip id check failed", __FILE__, __FUNCTION__, __LINE__);
    return false;
  }

  data = REG_BANK_SEL_RESET_VALUE;
  ret = i2c_.I2cWriteBytes(SLAVE_ADDR, REG_BANK_SEL, &data, 1);  // 选择bank0
  if (!ret) {
    HJ_ERROR("file:%s, FUNC:%s, line:%d, select bank0 failed", __FILE__, __FUNCTION__, __LINE__);
    return false;
  }


  data = DEVICE_CONFIG_RESET_FLAG;
  ret = i2c_.I2cWriteBytes(SLAVE_ADDR, DEVICE_CONFIG, &data, 1);  // 软复位
  if (!ret) {
    HJ_ERROR("file:%s, FUNC:%s, line:%d, soft reset failed", __FILE__, __FUNCTION__, __LINE__);
    return false;
  }
  usleep(10 * 1000);


  // gyro配置
  data = GYRO_CONFIG0_RESET_VALUE;
  ret = i2c_.I2cWriteBytes(SLAVE_ADDR, GYRO_CONFIG0, &data, 1);
  if (!ret) {
    HJ_ERROR("file:%s, FUNC:%s, line:%d, set gyro mode failed", __FILE__, __FUNCTION__, __LINE__);
    return false;
  }

  usleep(10 * 1000);  // TODO(wh): 确认是否需要延时，定义宏
  // acc配置
  data = ACCEL_CONFIG0_RESET_VALUE;
  ret = i2c_.I2cWriteBytes(SLAVE_ADDR, ACCEL_CONFIG0, &data, 1);
  if (!ret) {
    HJ_ERROR("file:%s, FUNC:%s, line:%d, set acc mode failed", __FILE__, __FUNCTION__, __LINE__);
    return false;
  }
  usleep(10 * 1000);


  data = PWR_MGMT0_RESET_VALUE;
  ret = i2c_.I2cWriteBytes(SLAVE_ADDR, PWR_MGMT0, &data, 1);
  if (!ret) {
    HJ_ERROR("file:%s, FUNC:%s, line:%d, set pw mode failed", __FILE__, __FUNCTION__, __LINE__);
    return false;
  }
  usleep(10 * 1000);
  return true;
}

void Imu::ImuTimer(const hj_bf::HJTimerEvent &) {
  static uint8_t buffer[16];

  int16_t acc_x = 0, acc_y = 0, acc_z = 0;
  float degrees_accx = 0.0f, degrees_accy = 0.0f, degrees_accz = 0.0f;
  float degrees_gyrox = 0.0f, degrees_gyroy = 0.0f, degrees_gyroz = 0.0f;
  int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;

  bool ret = i2c_.I2cReadBytes(SLAVE_ADDR, ACCEL_DATA_X1, buffer, READ_DATA_LEN);
  if (ret) {
    acc_x = static_cast<int16_t>(static_cast<int16_t>(buffer[0] << 8) | static_cast<int16_t>(buffer[1]));
    acc_y = static_cast<int16_t>(static_cast<int16_t>(buffer[2] << 8) | static_cast<int16_t>(buffer[3]));
    acc_z = static_cast<int16_t>(static_cast<int16_t>(buffer[4] << 8) | static_cast<int16_t>(buffer[5]));

    degrees_accx = acc_x/ACCEL_SENSITIVITY_SCALE_FACTOR;
    degrees_accy = acc_y/ACCEL_SENSITIVITY_SCALE_FACTOR;
    degrees_accz = acc_z/ACCEL_SENSITIVITY_SCALE_FACTOR;
    degrees_accx *= GYRO_ACC_ODR;
    degrees_accy *= GYRO_ACC_ODR;
    degrees_accz *= GYRO_ACC_ODR;

    gyro_x = static_cast<int16_t>(static_cast<int16_t>(buffer[6] << 8) | static_cast<int16_t>(buffer[7]));
    gyro_y = static_cast<int16_t>(static_cast<int16_t>(buffer[8] << 8) | static_cast<int16_t>(buffer[9]));
    gyro_z = static_cast<int16_t>(static_cast<int16_t>(buffer[10] << 8) | static_cast<int16_t>(buffer[11]));

    degrees_gyrox = gyro_x/GYRO_SENSITIVITY_SCALE_FACTOR;
    degrees_gyroy = gyro_y/GYRO_SENSITIVITY_SCALE_FACTOR;
    degrees_gyroz = gyro_z/GYRO_SENSITIVITY_SCALE_FACTOR;
    // 乘以π 除以180 转化为弧度
    degrees_gyrox = (degrees_gyrox * PI_DEFINE)/DEGREE;
    degrees_gyroy = (degrees_gyroy * PI_DEFINE)/DEGREE;
    degrees_gyroz = (degrees_gyroz * PI_DEFINE)/DEGREE;

    imu_msg_.accel_x = degrees_accx;
    imu_msg_.accel_y = degrees_accy;
    imu_msg_.accel_z = degrees_accz;
    imu_msg_.gyro_x = degrees_gyrox;
    imu_msg_.gyro_y = degrees_gyroy;
    imu_msg_.gyro_z = degrees_gyroz;
    imu_msg_.timestamp = ros::Time::now();

    if (!status_) {
      srv_msg_.request.code_val = SOC_IMU_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::NORMAL;
      hj_bf::HjPushSrv(srv_msg_);
    }
    status_ = true;
    error_count_ = 0;
    chatter_pub_.publish(imu_msg_);
  } else {
    if (status_ && error_count_ > ERROR_COUNT) {
      srv_msg_.request.code_val = SOC_IMU_DATA_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg_);
      status_ = false;
      HJ_ERROR("imuTimer rd status failed\n");
    }
    error_count_++;
  }
}

void Imu::RestartCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != 0 && !init_status_) {
    HJ_INFO("restart imu");
    Start();
  }
}

bool Imu::Start() {
  if (!i2c_.Initialize()) {  // TODO(wh): 确认返回值bool
    HJ_ERROR("can not open file %s", DEV_PATH);
    init_status_ = false;
    srv_msg_.request.code_val = SOC_IMU_INIT_ERROR;
    srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
    hj_bf::HjPushSrv(srv_msg_);
    return false;
  } else {
    if (!Iim42652Init()) {
      HJ_ERROR("Iim42652Init error");
      init_status_ = false;
      srv_msg_.request.code_val = SOC_IMU_INIT_ERROR;
      srv_msg_.request.status = hj_interface::HealthCheckCodeRequest::FAILED;
      hj_bf::HjPushSrv(srv_msg_);
      return false;
    } else {
      init_status_ = true;
      hj_bf::HJTimerEvent t_event;
      std::function<void(void)> sub_callback = std::bind(&Imu::ImuTimer, this, t_event);
      high_pef_timer_.start(frequency_, sub_callback);
    }
  }
  return true;
}

Imu::~Imu() {
  HJ_INFO("~imu");
}

Imu::Imu(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  if (json_conf.HasMember("frequency") && json_conf["frequency"].IsFloat()) {
    frequency_ = json_conf["frequency"].GetFloat();
  }
  if (json_conf.HasMember("dev") && json_conf["dev"].IsString()) {
    std::string dev_path = json_conf["dev"].GetString();
    if (!dev_path.empty()) {
      i2c_.SetDev(dev_path);
    }
  }

  chatter_pub_ = hj_bf::HJAdvertise<hj_interface::SocImu>("soc_imu_chatter", 10);
  restart_sub_ = hj_bf::HJSubscribe("/xxx", 1, &Imu::RestartCallback, this);

  // start
  if (Start()) {
    HJ_INFO("imu constructor OK");
  } else {
    HJ_ERROR("imu start failed");
  }
}
}  // namespace collect_node_imu
