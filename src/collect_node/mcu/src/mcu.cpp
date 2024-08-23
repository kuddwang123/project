#include "mcu.h"
#include "log.h"
#include <cerrno>
#include <fcntl.h>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include "hjlog.h"
#include "node_cache.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "hj_interface/StationUltra.h"
#include "hj_interface/PumpMotorSpeed.h"
#include "hj_interface/FanMotorSpeed.h"
#include <boost/filesystem.hpp>

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<collect_node_mcu::Mcu>(FUNCTION_NAME);
}

namespace collect_node_mcu {
std::string g_machine_version = "P3.5";

Queue::Queue(unsigned int size)
    : size_(size), head_(0), tail_(0), buffer_(nullptr) {
  if (size_ > 0) {
    buffer_ = new uint8_t[size_];
  }
  else {
    buffer_ = new uint8_t;
    size_ = 1;
  }
  assert(buffer_);
}

Queue::Queue(const Queue &other) {
  if (other.buffer_ == this->buffer_) {
    return;
  }
  if (other.buffer_ == nullptr) {
    buffer_ = nullptr;
    size_ = 0;
    return;
  }

  buffer_ = new uint8_t[other.size_];
  size_ = other.size_;
  memcpy(buffer_, other.buffer_, sizeof(uint8_t) * size_);
}

Queue &Queue::operator=(const Queue &other) {
  if (this->buffer_ == other.buffer_) {
    return *this;
  }

  delete[] this->buffer_;

  buffer_ = new uint8_t[other.size_];
  size_ = other.size_;
  memcpy(buffer_, other.buffer_, sizeof(uint8_t) * size_);

  return *this;
}

Queue::~Queue() {
    delete[] buffer_;
}

bool Queue::enqueue(uint8_t data) {
  if (isFull()) {
    HJ_DEBUG("Queue is full!\n");
    return false;
  }

  buffer_[tail_] = data;
  tail_ = (tail_ + 1) % size_;
  return true;
}

uint8_t Queue::dequeue() {
  if (isEmpty()) {
    HJ_DEBUG("Queue is empty!\n");
    return -1;
  }
  uint8_t data = buffer_[head_];
  head_ = (head_ + 1) % size_;
  return data;
}

bool Queue::isFull() { return (tail_ + 1) % size_ == head_; }

bool Queue::isEmpty() { return head_ == tail_; }

UartDataHandler::UartDataHandler()
    : uart_lastByte_(0), uart_beginFlag_(false), uart_ctrlFlag_(false),
      uart_revOffset_(0), checkSum_(0), rxPackageDataCount_(0), time_update_(0),
      time_diff_(0), motorcnt_(0), imucnt_(0), keylogDir_("/userdata/hj/log/mcukey"), 
      keylog_(nullptr) {}

UartDataHandler::~UartDataHandler() {
  hj_cst_log_del(keylog_);
  delete uart_;
}

bool UartDataHandler::initialize(const std::string& dev,
                          int baud, int flowctl, int databit, 
                          int stopbit, int parity, bool keylog) {
  memset(m_pRxBuf, 0, MAX_PACKAGE_SIZE);
  memset(u_recvBuf_, 0, MAX_PACKAGE_SIZE);
  if (keylog) {
    std::string keyfile = keylogDir_ + "/key.log";
    if (!boost::filesystem::is_directory(keylogDir_.data())) {
        if (!boost::filesystem::create_directories(keylogDir_.data())) {
            HJ_ERROR("create dir %s error\n", keylogDir_.c_str());
        } else {
            keylog_ = hj_cst_log_add(keyfile.data(), INFO_LOG, 10*1024*1024, 5);
        }
    } else {
        keylog_ = hj_cst_log_add(keyfile.data(), INFO_LOG, 10*1024*1024, 5);
    }
  }

  uart_ = new hj_bf::Uart(dev);
  assert(uart_);
  uart_->initialize(baud, flowctl, databit, stopbit, parity);
  return uart_->run();
}

void UartDataHandler::reset() {
  uart_lastByte_ = 0;
  uart_beginFlag_ = false;
  uart_ctrlFlag_ = false;
  uart_revOffset_ = 0;
  checkSum_ = 0; 
  rxPackageDataCount_ = 0;
  time_update_ = 0;
  time_diff_ = 0;
  motorcnt_ = 0;
  imucnt_ = 0;

  memset(m_pRxBuf, 0, MAX_PACKAGE_SIZE);
  memset(u_recvBuf_, 0, MAX_PACKAGE_SIZE);

  if (uart_ != nullptr) {
    uart_->flushin();
  }
}

void UartDataHandler::writePackage2Uart(uint8_t *buf, uint8_t len) {
  int8_t i = 0;
  uint8_t *pBuf = nullptr;
  uint8_t SendBuf[64];
  uint8_t sendlen = 0;
  uint8_t CheckSum = 0;

  pBuf = SendBuf;
  *pBuf++ = FRAMEHEAD;
  *pBuf++ = FRAMEHEAD;

  for (i = 0; i < len; i++) {
    if ((buf[i] == static_cast<uint8_t>(FRAMECTRL)) || 
        (buf[i] == static_cast<uint8_t>(FRAMEHEAD)) ||
        (buf[i] == static_cast<uint8_t>(FRAMETAIL))) {
      *pBuf++ = FRAMECTRL;
    }
    *pBuf++ = buf[i];
    CheckSum += buf[i];
  }

  // checksum
  if ((CheckSum == static_cast<uint8_t>(FRAMECTRL)) || 
      (CheckSum == static_cast<uint8_t>(FRAMEHEAD)) ||
      (CheckSum == static_cast<uint8_t>(FRAMETAIL))) {
    *pBuf++ = FRAMECTRL;
  }
  *pBuf++ = CheckSum;

  // Send Tail USART_FRAMETAIL USART_FRAMETAIL
  *pBuf++ = FRAMETAIL;
  *pBuf++ = FRAMETAIL;

  sendlen = pBuf - SendBuf;

  uart_->send(SendBuf, sendlen);
}

bool UartDataHandler::analyzePackage(McuImpl* mcuImpl) {
  int byte_num = uart_->recv(u_recvBuf_);
  if (byte_num <= 0) {
    HJ_ERROR("no read from uart\n");
    return false;
  }

  uint8_t data = 0;
  
  for (int i = 0; i < byte_num; i++) {
    data = u_recvBuf_[i];
    if (((data == FRAMEHEAD) && (uart_lastByte_ == FRAMEHEAD)) ||
        (uart_revOffset_ > MAX_PACKAGE_SIZE)) {
      // RESET
      uart_revOffset_ = 0;
      uart_beginFlag_ = true;
      uart_lastByte_ = data;
      continue;
    }
    if ((data == FRAMETAIL) && (uart_lastByte_ == FRAMETAIL) && uart_beginFlag_) {
      uart_revOffset_--;
      rxPackageDataCount_ = uart_revOffset_ - 1;
      checkSum_ -= FRAMETAIL;
      checkSum_ -= m_pRxBuf[rxPackageDataCount_];
      uart_lastByte_ = data;
      uart_beginFlag_ = false;
      
      if (checkSum_ == m_pRxBuf[rxPackageDataCount_]) {
        uint32_t id = *(reinterpret_cast<uint32_t*>(m_pRxBuf));

        switch (id) {
            case MOTOR_MSG_ID: {
                Motor_msg* motor_msg = reinterpret_cast<Motor_msg*>(m_pRxBuf);
                if (time_update_ == 0) {
                    time_diff_ = ros::Time::now().toSec() * 1000 - motor_msg->time_ms;
                    time_update_ = 1;
                }

                hj_interface::Encoder msg;
                msg.custom_time = 
                    ros::Time().fromSec((time_diff_ + motor_msg->time_ms) / 1000.0l);
                msg.left_msg = motor_msg->lw_speed / 1.0l;
                msg.right_msg = motor_msg->rw_speed / 1.0l;
                msg.index = ++motorcnt_;
                mcuImpl->motorPub(msg);
#ifdef DEBUG
                hj_interface::Atime time_msg;
                time_msg.timestamp_current = ros::Time().now();
                time_msg.timestamp_origin = msg.custom_time;
                time_msg.index = msg.index;
                mcuImpl->motorTimePub(time_msg);
#endif
            }
            break;

            case IMU_MSG_ID: {
                IMU_msg *msg = reinterpret_cast<IMU_msg*>(m_pRxBuf);
                if (time_update_ == 0) {
                    time_diff_ = ros::Time::now().toSec() * 1000 - msg->time_ms;
                    time_update_ = 1;
                    mcuImpl->pubFirstImuTime(msg->time_ms);
                    mcuImpl->pubTimeDiff(time_diff_sum_);
                }
                if (time_update_ >= 6000) {  // 1 minuites sync
                    double now = ros::Time::now().toSec() * 1000;
                    time_diff_sum_ = now - static_cast<double>(msg->time_ms) - time_diff_;
                    time_update_ = 1;
                    HJ_INFO("time_diff: %lf, now: %lf, msg->time_ms: %d, diff: %lf\n", time_diff_, now, msg->time_ms, time_diff_sum_);
                    mcuImpl->pubTimeDiff(time_diff_sum_);
                }
                time_update_++;

                hj_interface::Imu Imu;
                Imu.custom_time =
                    ros::Time().fromSec((time_diff_ + msg->time_ms) / 1000.0l);

                Imu.roll = msg->roll;
                Imu.pitch = msg->pitch;
                Imu.yaw = msg->yaw;
                //线加速度
                Imu.accel_x = msg->accx;
                Imu.accel_y = msg->accy;
                Imu.accel_z = msg->accz;
                //角速度
                Imu.gyro_x = msg->gyrox;
                Imu.gyro_y = msg->gyroy;
                Imu.gyro_z = msg->gyroz;
#ifdef DEBUG
                Imu.index = ++imucnt_;
                mcuImpl->imuPub(Imu);
                hj_interface::Atime time_msg;
                time_msg.timestamp_current = ros::Time().now();
                time_msg.timestamp_origin = Imu.custom_time;
                time_msg.index = Imu.index;
                time_msg.flag = msg->flag;
                mcuImpl->imuTimePub(time_msg);
#endif
            }
            break;

            case BAT_MSG_ID: {
                BAT_msg *msg = reinterpret_cast<BAT_msg*>(m_pRxBuf);
                
                hj_interface::Bat bat_msg;
                bat_msg.power = msg->power;
                bat_msg.temp1 = msg->temp1;
                bat_msg.temp2 = msg->temp2;
                bat_msg.temp3 = msg->temp3;
                bat_msg.bat_vol = msg->bat_vol;
                bat_msg.bat_disch_cur = msg->bat_disch_cur;
                bat_msg.ch_vol = msg->ch_vol;
                bat_msg.charger_ch_cur = msg->charger_ch_cur;
                bat_msg.bat_cycle_times = msg->cycle_times;
                bat_msg.bat_health_left = msg->health_left;

                mcuImpl->batPub(bat_msg);
            }
            break;

            case TEMP_HUMIDITY_ID: {
                Temp_Humidity* msg = reinterpret_cast<Temp_Humidity*>(m_pRxBuf);
                hj_interface::TempHumidity tempHumidity_msg;
                tempHumidity_msg.humidity = msg->humidity;
                tempHumidity_msg.temperature = msg->temperature;
                // HJ_INFO( "humidity:%d, temperature:%d :\n", tempHumidity_msg.humidity,
                //                   tempHumidity_msg.temperature);
                mcuImpl->tempHumidityPub(tempHumidity_msg);
            }
            break;
    
            case REMOTE_CONTROL_ID: {
                HJ_INFO("remote ctl: %02x %02x %02x %02x %02x\n",
                  m_pRxBuf[4],
                  m_pRxBuf[5],
                  m_pRxBuf[6],
                  m_pRxBuf[7],
                  m_pRxBuf[8]);

                uint8_t flag = m_pRxBuf[4];
                if (flag == 1) {
                    uint32_t control = *(reinterpret_cast<uint32_t*>(m_pRxBuf+5));
                    mcuImpl->pubTmtCltToMid(control);
                }
            }
            break;

            case MCU_KEY_LOG: {
                uint16_t len = 0;
                len = m_pRxBuf[rxPackageDataCount_-5];
                char* key = new char[len*3+1];
                memset(key, 0x00, len*3+1);
                //char key[len*3+1] = {0x00};
                int written = 0;
                for (uint16_t i = 4; i < len+4; i++) {
                    written += snprintf(key+written, 4, "%02x ", m_pRxBuf[i]);
                }
                MCU_KEYLOG(keylog_, "%s\n", key);
                delete[] key;
            }
            break;

            case MCU_SELF_CHECK: {
                uint8_t type = m_pRxBuf[4];
                uint8_t status = m_pRxBuf[5];
                switch (type) {
                    case 0: {  // [0x00]电机
                        ElectricalMachineryStatus *elecStatus = reinterpret_cast<ElectricalMachineryStatus*>(m_pRxBuf);
                        HJ_INFO("MCU_SELF_CHECK ElectricalMachineryStatus: %d, %d, %d, %d, %d, %d,  %d,%d\n",
                        elecStatus->left_wheel_status, elecStatus->rignt_wheel_status,
                        elecStatus->steering_motor_status, elecStatus->left_water_pump_status,
                        elecStatus->right_water_pump_status, elecStatus->airbag_status, elecStatus->fan_status,
                        elecStatus->flip_cover_motor_status);

                        if (status == 1) {
                            mcuImpl->setHealthError(2000, hj_interface::HealthCheckCodeRequest::FAILED);
                        }
                        mcuImpl->selfHealthCheckAck(MOTOR);
                        break;
                    }
                    case 1: {  // [0x01]IMU
                        HJ_INFO("MCU_SELF_CHECK imu_status: %d\n", status);
                        
                        if (status == 1) {
                            mcuImpl->setHealthError(2001, hj_interface::HealthCheckCodeRequest::FAILED);
                        }
                        mcuImpl->selfHealthCheckAck(IMU);
                        break;
                    }
                    case 2: {  // [0x02]BMS
                        HJ_INFO("MCU_SELF_CHECK bms_status: %d\n", status);
                        if (status == 1) {
                            mcuImpl->setHealthError(2002, hj_interface::HealthCheckCodeRequest::FAILED);
                        }
                        mcuImpl->selfHealthCheckAck(BMS);
                        break;
                    }
                    case 3: {  // [0x03]LORA模组
                        if (status == 1) {
                            mcuImpl->setHealthError(2003, hj_interface::HealthCheckCodeRequest::FAILED);
                        }
                        if (status != 2) {
                            mcuImpl->selfHealthCheckAck(LORA);
                        }

                        // HJ_INFO("MCU_SELF_CHECK lora_status: %d\n", status);
                        break;
                    }
                    default: {
                        HJ_ERROR("MCU_SELF_CHECK: MCU_FAIL_CODE: type error,type=%d\n", type);
                        break;
                    }
                }
                
            }
            break;

            case MCU_FAIL_CODE: {
                uint8_t fail_code_count = m_pRxBuf[4];
                for (uint8_t i = 0; i < fail_code_count; i++) {
                    uint8_t fail_type = m_pRxBuf[i * 3 + 5];
                    uint16_t fail_code = static_cast<uint16_t>(m_pRxBuf[i * 3 + 6] << 8 | m_pRxBuf[i * 3 + 7]);
                    mcuImpl->setHealthError(fail_code, fail_type);
                    HJ_INFO("MCU_FAIL_CODE fail_type:%d, fail_code:%d\n", fail_type, fail_code);
                }
            }
            break;

            case MCU_ONEKEY_TRIGGER: {
                Key_msg* key_msg = reinterpret_cast<Key_msg*>(m_pRxBuf);
                // HJ_INFO("key:%d %d\n", key_msg->flag, key_msg->value);
               
                mcuImpl->pubKeyCtlToMid(key_msg->flag, key_msg->value);
            }
            break;

            case MCU_E_MOTOR: {
                Emotor_Cur_msg* cmsg = reinterpret_cast<Emotor_Cur_msg*>(m_pRxBuf);
                hj_interface::ElecMotorCur msg;
                msg.custom_time = ros::Time::now();
                msg.motor_l = cmsg->motor_l;
                msg.motor_r = cmsg->motor_r;
                msg.pump_l = cmsg->pump_l;
                msg.pump_r = cmsg->pump_r;
                msg.turn = cmsg->turn;
                msg.dirtybox = cmsg->dirtybox;
                msg.flipcover = cmsg->flipcover;
                msg.airbag = cmsg->airbag;
                mcuImpl->mototCurPub(msg);
            }
            break;

            case DOWNWARD_SENSOR_ID: {
#ifdef HJ_T1pro
                hj_interface::DownRay msg;
                uint8_t left = *(m_pRxBuf+4);
                uint8_t right = *(m_pRxBuf+5);
                msg.left_down = left;
                msg.right_down = right;
                msg.timestamp = ros::Time::now();
                mcuImpl->InfraredSensorPub(msg);
#else
                if (g_machine_version == "P3.5") {
                  hj_interface::DownLeft msg;
                  uint8_t left = *(m_pRxBuf+4);
                  msg.ray_value = left;
                  msg.timestamp = ros::Time::now();
                  mcuImpl->InfraredSensorPub(msg);
                } else if (g_machine_version == "P4") {
                } else {
                  hj_interface::DownRay msg;
                  uint8_t left = *(m_pRxBuf+4);
                  uint8_t right = *(m_pRxBuf+5);
                  msg.left_down = left;
                  msg.right_down = right;
                  msg.timestamp = ros::Time::now();
                  mcuImpl->InfraredSensorPub(msg);
                }
#endif
            }
            break;

            case OUTWATER_DETEC_ID: {
                uint8_t state = *(m_pRxBuf+4);
                // HJ_INFO("out water state:%d\n", state);
                mcuImpl->outwaterPub(state);
            }
            break;

            case LORA_TRANSPORT_ID: {
                Lora_header* header = reinterpret_cast<Lora_header*>(m_pRxBuf);
                if (header->key1 == 'M' && header->key2 == 'R') {
                    Station_Ultra_msg* ultra = reinterpret_cast<Station_Ultra_msg*>(m_pRxBuf);
                    /*
                    HJ_INFO("ultra flag receive:%d\n", ultra->flag);
                    HJ_INFO("left:%d  %d  %d  %d  %d\n", 
                            ultra->ultra_lt[0],
                            ultra->ultra_lt[1],
                            ultra->ultra_lt[2],
                            ultra->ultra_lt[3],
                            ultra->ultra_lt[4]);
                    HJ_INFO("right:%d  %d  %d  %d  %d\n", 
                            ultra->ultra_rt[0],
                            ultra->ultra_rt[1],
                            ultra->ultra_rt[2],
                            ultra->ultra_rt[3],
                            ultra->ultra_rt[4]);
                    */
                    if (ultra->flag == 1) {
                        mcuImpl->stationUltraPub(ultra);
                    }
                }
            }
            break;

            case MCU_VER_ACK: {
                Mcu_Ver_msg* msg = reinterpret_cast<Mcu_Ver_msg*>(m_pRxBuf);
                mcuImpl->stopGetVerTimer();
                mcuImpl->writePubMcuVersion(msg);
            }
            break;

            case RETURN_CHARGER_ID: {
                uint8_t state = *(m_pRxBuf+4);
                mcuImpl->onChargerPub(state);
            }
            break;

            case DIRTYBOX_DETECT_ID: {
                uint8_t state = *(m_pRxBuf+4);
                mcuImpl->pubDirtBoxToMid(state);
            }
            break;

            case PUMP_MOTOR_SPEED: {
                Pump_Motor_Speed_msg* msg = reinterpret_cast<Pump_Motor_Speed_msg*>(m_pRxBuf);
                mcuImpl->pubPumpMotorSpeed(msg->speed_l, msg->speed_r);
            }
            break;

            case FAN_MOTOR_SPEED: {
                Fan_Motor_Speed_msg* msg = reinterpret_cast<Fan_Motor_Speed_msg*>(m_pRxBuf);
                mcuImpl->pubFanMotorSpeed(msg->speed);
            }
            break;
#ifdef HJ_T1pro
            case DTOF_SENSOR_ID: {
                hj_interface::LeftTof msg;
                Tof_Sensor_msg* data = reinterpret_cast<Tof_Sensor_msg*>(m_pRxBuf);
                uint16_t front = data->dist_front;  // 单位: mm
                uint16_t left = data->dist_left;  // 单位: mm
                if (front > 500) {  // tof 有效量程为 1~500mm
                  msg.dist_front = 500;
                } else {
                  msg.dist_front = front;
                }
                if (left > 500) {
                  msg.dist_back = 500;
                } else {
                  msg.dist_back = left;
                }
                double time_now_ms = ros::Time::now().toSec() * 1000.0;
                msg.timestamp = ros::Time().fromSec((time_now_ms - time_diff_sum_) / 1000.0);
                mcuImpl->TofPub(msg);
            }
            break;

            case TURBIDITY_SENSOR_ID: {
              hj_interface::Turbidity msg;
              uint8_t turbidity = *(m_pRxBuf + 4);
              msg.timestamp = ros::Time::now();
              msg.turbidity = turbidity;
              mcuImpl->TurbidityPub(msg);
            }
            break;

            case DUST_PLUG_DETECTION: {
              hj_interface::DustPlugDetection msg;
              uint8_t status1 = *(m_pRxBuf + 4);
              uint8_t status2 = *(m_pRxBuf + 5);
              msg.timestamp = ros::Time::now();
              msg.status1 = status1;
              msg.status2 = status2;
              mcuImpl->DustPlugPub(msg);
            }
            break;
#else
            case DTOF_SENSOR_ID: {
              if (g_machine_version == "P4") {
                hj_interface::DownLeft msg;
                Tof_Sensor_msg* data = reinterpret_cast<Tof_Sensor_msg*>(m_pRxBuf);
                uint16_t left = data->dist_left;  // 单位: mm
                if (left > 500) {  // tof 有效量程为 1~500mm
                  msg.ray_value = 500;
                } else {
                  msg.ray_value = left;
                }
                double time_now_ms = ros::Time::now().toSec() * 1000.0;
                msg.timestamp = ros::Time().fromSec((time_now_ms - time_diff_sum_) / 1000.0);
                mcuImpl->InfraredSensorPub(msg);
              }
            }
            break;
#endif
            case RTC_TIMESYNC_ACK: {
                Rtc_TimeSync_Ack_msg* msg = reinterpret_cast<Rtc_TimeSync_Ack_msg*>(m_pRxBuf);
                HJ_INFO("rtc time sync:%d\n", msg->result);
                if (msg->result == 1) {
                    mcuImpl->stopMcuTimeSync();
                }
            }
            break;

            case MOTOR_MSG_ACK:
            case IMU_CALI_ACK:
            case AIRBAG_CTL_ACK:
            case FLIP_COVER_ACK:
            case PUMP_MSG_ACK:
            case TURN_MOTOR_ACK:
            case TURBIN_CTL_ACK:
            case MCU_RESET_ACK:
            case LIGHT_STRIP_ACK:
            case LIGHT_PEARL_ACK: {
                mcuImpl->sensorCtlAck(id, m_pRxBuf + 4);
                break;
            }

            default:
                ;
        }

        checkSum_ = 0;
        continue;
      }
      HJ_ERROR("CheckSum error \r\n");
      checkSum_ = 0;
      continue;
    }
    uart_lastByte_ = data;

    if (uart_beginFlag_) {
      if (uart_ctrlFlag_) {
        m_pRxBuf[uart_revOffset_++] = data;
        checkSum_ += data;
        uart_ctrlFlag_ = false;
        uart_lastByte_ = FRAMECTRL;
      } else if (data == FRAMECTRL) {
        uart_ctrlFlag_ = true;
      } else {
        m_pRxBuf[uart_revOffset_++] = data;
        checkSum_ += data;
      }
    }

    if (uart_revOffset_ >= MAX_PACKAGE_SIZE - 1) {
      uart_beginFlag_ = false;
      uart_revOffset_ = 0;
    }
  }

  return true;
}

SensorCtl::SensorCtl()
    :id_(0),
    len_(0),
    head_{0xff, 0xff, 0xff, 0xff},
    tail_{0xff, 0xff, 0xff, 0xff, 0xff},
    needCheckAck_(true),
    needCmpLast_(true),
    uartdata_(nullptr),
    atmoicFlag_(ATOMIC_FLAG_INIT) {
    
}

SensorCtl::~SensorCtl() {
    delete[] uartdata_;
}

void SensorCtl::init(uint32_t id, uint16_t len, UartDataHandler* uarthl, 
        const std::string& tmrName, bool needCheckAck, bool needCmpLast) {
    id_ = id;
    len_ = len;
    needCheckAck_ = needCheckAck;
    needCmpLast_ = needCmpLast;
    assert(uarthl);
    uarthl_ = uarthl;
    ackTmr_ = hj_bf::HJCreateTimer(tmrName, 100 * 1000, &SensorCtl::ackTimerCb, this, false);

    UINT32_TO_BUF_LITTLE(id_, head_);
    UINT16_TO_BUF_LITTLE(len_, tail_);

    uartdata_ = new uint8_t[sizeof(head_)/sizeof(uint8_t) +
                sizeof(tail_)/sizeof(uint8_t) +
                len_];
    memcpy(uartdata_, head_, sizeof(head_)/sizeof(uint8_t));
    memset(uartdata_ + sizeof(head_)/sizeof(uint8_t), 0xff, len_);
    memcpy(uartdata_ + sizeof(head_)/sizeof(uint8_t) + len_, tail_, sizeof(tail_)/sizeof(uint8_t));
}

void SensorCtl::pushData(const std::vector<uint8_t>& data) {
    if (needCmpLast_) {
        if (data == lastwrit_) {
            return;
        }
    }
    
    uartdata udata(data);
    
    writeUart(udata);

    spinLock();
    if (!queue_.empty()) {
        queue_.pop_front();
    }
    queue_.push_back(udata);
    spinUnLock();
    
    ackTmr_.start();
}

void SensorCtl::ack(uint8_t* ackdata) {
    std::vector<uint8_t> data;
    data.assign(len_, 0);
    ::memcpy(data.data(), ackdata, len_);

    spinLock();
    HJ_INFO("enter ack\n");
    if (queue_.empty()) {
        spinUnLock();
        return;
    }

    uartdata& udata = queue_.front();
    if (needCheckAck_) {
        if (udata._data != data) {
            HJ_ERROR("%d ack not equal\n", id_);
            HJ_INFO("ack:\n");
            for (const auto& v: data) {
                HJ_INFO("%02x ", v);
            }
            HJ_INFO("old:\n");
            for (const auto& v: udata._data) {
                HJ_INFO("%02x ", v);
            }
        } else {
            udata._acked = true;
        }
    } else {
        udata._acked = true;
    }
    
    if (udata._acked) {
        lastwrit_ = udata._data;
        queue_.pop_front();
    }

    spinUnLock();
}

void SensorCtl::writeUart(uartdata& data) {
    memcpy(uartdata_ + sizeof(head_)/sizeof(uint8_t), data._data.data(), len_);
    HJ_INFO("send uart:%02x", id_);
    for (const auto& v: data._data) {
        HJ_INFO("%02x ", v);
    }
    uarthl_->writePackage2Uart(uartdata_, sendDataLen());
    --data._maxcnt;
}

uint16_t SensorCtl::sendDataLen() {
    return sizeof(head_)/sizeof(uint8_t) +
        sizeof(tail_)/sizeof(uint8_t) +
        len_;
}

uint16_t SensorCtl::dataLen() {
    return len_;
}

void SensorCtl::ackTimerCb(const hj_bf::HJTimerEvent&) {
    spinLock();
    if (queue_.empty()) {
        spinUnLock();
        ackTmr_.stop();
        return;
    }

    uartdata& udata = queue_.front();
    if (udata._maxcnt == 0) {
        HJ_ERROR("send %02x time out!\n", id_);
        queue_.pop_front();
        spinUnLock();
        return;
    }

    writeUart(udata);
    spinUnLock();
}

void SensorCtl::spinLock() {
    while (atmoicFlag_.test_and_set(std::memory_order_acquire)) {}
}

void SensorCtl::spinUnLock() {
    atmoicFlag_.clear(std::memory_order_release);
}

McuImpl::McuImpl(std::string dev)
    : dev_(dev), 
      isRun_(false),
      motor_health_ack_(0),
      imu_health_ack_(0),
      bms_health_ack_(0),
      lora_health_ack_(0) {}

McuImpl::~McuImpl() {
  isRun_ = false;
  if (dealUartThred_.joinable()) {
    dealUartThred_.join();
  }
  delete uartDataHandler_;
}

bool McuImpl::run(int baud, int flowctl, int databit, int stopbit, int parity, bool keylog) {
  uartDataHandler_ = new UartDataHandler;
  assert(uartDataHandler_);
  
  if (!uartDataHandler_->initialize(dev_, baud, flowctl, databit, stopbit, parity, keylog)) {
    HJ_ERROR("uart init fail!\n");
    return false;
  }
  
  initSensorCtl();

#ifdef DEBUG
  motor_time_pub_ = hj_bf::HJAdvertise<hj_interface::Atime>("motor_time_chatter", 10);
  imu_time_pub_ = hj_bf::HJAdvertise<hj_interface::Atime>("imu_time_chatter", 10);
#endif
  motor_pub_ = hj_bf::HJAdvertise<hj_interface::Encoder>("motor_chatter", 10);
  imu_pub_ = hj_bf::HJAdvertise<hj_interface::Imu>("imu_chatter", 10);
  bat_pub_ = hj_bf::HJAdvertise<hj_interface::Bat>("bat_chatter", 10);
  mcuota_pub_ = hj_bf::HJAdvertise<std_msgs::Bool>("mcuOtaReady", 1);
  motot_cur_pub_ = hj_bf::HJAdvertise<hj_interface::ElecMotorCur>("/motor_cur", 1);
  outof_water_pub_ = hj_bf::HJAdvertise<std_msgs::UInt8>("/outwater_detect", 1);
  station_ultra_pub_ = hj_bf::HJAdvertise<hj_interface::StationUltra>("/station_ultra", 10);
  to_middleware_pub_ = hj_bf::HJAdvertise<std_msgs::String>("/to_middle", 5);
  tempHumidity_pub_ = hj_bf::HJAdvertise<hj_interface::TempHumidity>("tempHumidity_chatter", 10);
  ret_charger_pub_ = hj_bf::HJAdvertise<std_msgs::UInt8>("/onCharger_chatter", 5);
  mcuVer_pub_ = hj_bf::HJAdvertise<std_msgs::String>("/mcuVer_chatter", 1, true);
  pump_motor_speed_pub_ = hj_bf::HJAdvertise<hj_interface::PumpMotorSpeed>("/pumpMotorSpeed_chatter", 1);
  fan_motor_speed_pub_ = hj_bf::HJAdvertise<hj_interface::FanMotorSpeed>("fanMotorSpeed_chatter", 1);
  first_imu_pub_ = hj_bf::HJAdvertise<std_msgs::UInt32>("/machine_on/times", 1, true);
  time_diff_pub_ = hj_bf::HJAdvertise<std_msgs::Float64>("/time_diff_chatter", 1, true);

#ifdef HJ_T1pro
  tof_pub_ = hj_bf::HJAdvertise<hj_interface::LeftTof>("t1pro/left_tof", 10);
  infrared_sensor_pub_ = hj_bf::HJAdvertise<hj_interface::DownRay>("t1pro/down_ray", 10);
  turbidity_pub_ = hj_bf::HJAdvertise<hj_interface::Turbidity>("turbidity_data", 10);
  dust_plug_pub_ = hj_bf::HJAdvertise<hj_interface::DustPlugDetection>("t1pro/dust_plug_detection_chatter", 10);
#else
  if (g_machine_version == "P3.5" || g_machine_version == "P4") {
    infrared_sensor_pub_ = hj_bf::HJAdvertise<hj_interface::DownLeft>("x9/down_left", 10);
  } else {
    infrared_sensor_pub_ = hj_bf::HJAdvertise<hj_interface::DownRay>("x9/down_ray", 10);
  }
#endif

  sub_motor_set_ = hj_bf::HJSubscribe("nav_motor", 1, 
                                      &McuImpl::motor_set_chatterCallback, this);
  sub_airbag_ = hj_bf::HJSubscribe("/air_bag", 1,
                                   &McuImpl::airbagCb, this);
  /*
  sub_steerpump_ = hj_bf::HJSubscribe("/steer_pump", 1,
                                   &McuImpl::steerPumpCb, this);
  */
  sub_flipcover_ = hj_bf::HJSubscribe("/flip_cover", 1,
                                   &McuImpl::flipCoverCb, this);
  sub_pumpsteer_ = hj_bf::HJSubscribe("/pump_steer", 1,
                                   &McuImpl::pumpSteerCb, this);
  sub_turbin_motor_ = hj_bf::HJSubscribe("/turbine_motor", 1,
                                   &McuImpl::turbinMotorCb, this);
  sub_imucal_ = hj_bf::HJSubscribe("/imu_work_model", 1,
                                   &McuImpl::imuWorkModeCb, this);
  sub_middleware_ = hj_bf::HJSubscribe("/to_collect", 10,
                                   &McuImpl::mcuCtlFromMiddleCb, this);
#ifdef X6
  sub_x6pump_ = hj_bf::HJSubscribe("/x6_pump", 1,
                                   &McuImpl::x6pumpCb, this);
#endif
  sub_mcuctl_ = hj_bf::HJSubscribe("mcuInterCtl", 1, 
                              &McuImpl::mcuCtlCallBack, this);
  
  sub_rtcTimeSync_ = hj_bf::HJSubscribe("/time/updatetime", 1,
                              &McuImpl::rtcTimeSyncCb, this);

  getMcuVerTmr_ = 
      hj_bf::HJCreateTimer("getMcuVersion", 500 * 1000, &McuImpl::getMcuVerTimerCb, this);
  getmcuLedVer_ =
      hj_bf::HJCreateTimer("getMcuLedVer", 500 * 1000, &McuImpl::getMcuLedVerTimerCb, this, false);
  rtcSyncTmr_ = 
      hj_bf::HJCreateTimer("rtcTimeSync", 500 * 1000, &McuImpl::rtcTimeSyncTimerCb, this, false);
  syncRtcWithFlagTmr_ = 
      hj_bf::HJCreateTimer("syncRtcWithFlag", 2 * 1000 * 1000, &McuImpl::syncRtcWithFlag, this);

  isRun_ = true;
  dealUartThred_ = std::thread(&McuImpl::dealUartLoop, this);

  beatMcu(); //先发送一次心跳
  heartBeat_ = hj_bf::HJCreateTimer("heartBeat", 900 * 1000,
                                    &McuImpl::heartBeatTimerCb, this);

  return true;
}

void McuImpl::initSensorCtl() {
    motorCtl_.init(0x10, 8, uartDataHandler_, "motor_ctl");
    airbagCtl_.init(0x34, 3, uartDataHandler_, "airbag_ctl", true, false);
    imuCalCtl_.init(0x14, 1, uartDataHandler_, "imucal_ctl");
    flipCtl_.init(0x38, 2, uartDataHandler_, "flip_ctl");
    pumpCtl_.init(0x32, 5, uartDataHandler_, "pump_ctl");
    turnCtl_.init(0x30, 3, uartDataHandler_, "turn_ctl");
    turbinCtl_.init(0x36, 3, uartDataHandler_, "turbin_ctl");
    powerCtl_.init(0x50, 1, uartDataHandler_, "power_ctl", false);
    lightStripCtl_.init(0x92, 2, uartDataHandler_, "lightStrip_ctl");
    lightPearlCtl_.init(0x94, 8, uartDataHandler_, "lightPearl_ctl");
}

void McuImpl::sensorCtlAck(uint32_t ackid, uint8_t* data) {
    HJ_INFO("ack id:%02x\n", ackid);
    switch(ackid) {
        case MOTOR_MSG_ACK:
            motorCtl_.ack(data);
            break;

        case AIRBAG_CTL_ACK:
            airbagCtl_.ack(data);
            break;
        
        case IMU_CALI_ACK:
            imuCalCtl_.ack(data);
            break;

        case FLIP_COVER_ACK:
            flipCtl_.ack(data);
            break;
        
        case PUMP_MSG_ACK:
            pumpCtl_.ack(data);
            break;
        
        case TURN_MOTOR_ACK:
            turnCtl_.ack(data);
            break;
        
        case TURBIN_CTL_ACK:
            turbinCtl_.ack(data);
            break;
        
        case MCU_RESET_ACK:
            powerCtl_.ack(data);
            break;
        
        case LIGHT_STRIP_ACK:
            lightStripCtl_.ack(data);
            break;
        
        case LIGHT_PEARL_ACK:
            lightPearlCtl_.ack(data);
            break;
        
        default:
            HJ_ERROR("unknown mcu ack:%02x\n", ackid);
    }
}

void McuImpl::mcuCtlCallBack(const std_msgs::Bool::ConstPtr& msg) {
  //true:打开串口读， false:关闭串口读
  if (msg->data == 0) {
    isRun_ = false;
    if(dealUartThred_.joinable()) {
      dealUartThred_.join();
    }
    uartDataHandler_->reset();

    std_msgs::Bool msg;
    msg.data = 1;
    mcuota_pub_.publish(msg);
  } else {
    isRun_ = true;
    dealUartThred_ = std::thread(&McuImpl::dealUartLoop, this);
  }
}

void McuImpl::dealUartLoop() {
  prctl(PR_SET_NAME, "dealUartLoop");
  uartDataHandler_->reset();
  while (isRun_) {
    uartDataHandler_->analyzePackage(this);
  }
}

void McuImpl::motorPub(const hj_interface::Encoder &msg) {
  motor_pub_.publish(msg);
}

void McuImpl::imuPub(const hj_interface::Imu &msg) {
  imu_pub_.publish(msg);
}
#ifdef DEBUG
void McuImpl::motorTimePub(const hj_interface::Atime &msg) {
  motor_time_pub_.publish(msg);
}

void McuImpl::imuTimePub(const hj_interface::Atime &msg) {
  imu_time_pub_.publish(msg);
}
#endif
void McuImpl::tempHumidityPub(const hj_interface::TempHumidity &msg) {
  tempHumidity_pub_.publish(msg);
}

#ifdef HJ_T1pro
void McuImpl::InfraredSensorPub(const hj_interface::DownRay &msg) {
  infrared_sensor_pub_.publish(msg);
}

void McuImpl::TofPub(const hj_interface::LeftTof& msg) {
  tof_pub_.publish(msg);
}
void McuImpl::TurbidityPub(const hj_interface::Turbidity& msg) {
  turbidity_pub_.publish(msg);
}
void McuImpl::DustPlugPub(const hj_interface::DustPlugDetection& msg) {
  dust_plug_pub_.publish(msg);
}
#else
void McuImpl::InfraredSensorPub(const hj_interface::DownLeft &msg) {
  infrared_sensor_pub_.publish(msg);
}

void McuImpl::InfraredSensorPub(const hj_interface::DownRay &msg) {
  infrared_sensor_pub_.publish(msg);
}
#endif

void McuImpl::batPub(const hj_interface::Bat &msg) {
  bat_pub_.publish(msg);
}

void McuImpl::onChargerPub(uint8_t state) {
  std_msgs::UInt8 pubmsg;
  pubmsg.data = state;
  ret_charger_pub_.publish(pubmsg);
  //HJ_INFO("on charger pub:%d\n", state);
}

void McuImpl::pubFirstImuTime(uint32_t ts) {
  std_msgs::UInt32 pubmsg;
  pubmsg.data = ts;
  first_imu_pub_.publish(pubmsg);
}

void McuImpl::pubTimeDiff(double diff_time) {
  std_msgs::Float64 pubmsg;
  pubmsg.data = diff_time;
  HJ_INFO("time diff:%f\n", diff_time);
  time_diff_pub_.publish(pubmsg);
}

void McuImpl::stationUltraPub(Station_Ultra_msg* msg) {
  hj_interface::StationUltra pubdata;
  uint16_t value = 0;
  for (int i = 0; i < 5; i++) {
    value = msg->ultra_lt[i];
    pubdata.ultra_lt.emplace_back(value);
  }

  for (int i = 0; i < 5; i++) {
    value = msg->ultra_rt[i];
    pubdata.ultra_rt.emplace_back(value);
  }

  //pubdata.ultra_lt.assign(msg->ultra_lt, msg->ultra_lt+5);
  //pubdata.ultra_rt.assign(msg->ultra_rt, msg->ultra_rt+5);
  station_ultra_pub_.publish(pubdata);
}

void McuImpl::outwaterPub(uint8_t value) {
  std_msgs::UInt8 pubmsg;
  pubmsg.data = value;
  outof_water_pub_.publish(pubmsg);
}

void McuImpl::pubKeyCtlToMid(uint8_t flag, uint8_t value) {
  rapidjson::Document document;
  document.SetObject();

  rapidjson::Value btn(rapidjson::kObjectType);
  btn.AddMember("pushTime", value, document.GetAllocator());
  btn.AddMember("type", flag, document.GetAllocator());
  document.AddMember("BTN", btn, document.GetAllocator());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer); 

  std_msgs::String msg;
  msg.data = std::string(buffer.GetString());
  to_middleware_pub_.publish(msg);

  HJ_INFO("to mid pub:\n%s\n", buffer.GetString());

  static uint8_t one_key_ack[] = {0x91, 0x0, 0x0, 0x0,
                          0x00, 0x00,
                          0xff, 0xff, 0xff};

  uartDataHandler_->writePackage2Uart(one_key_ack,
                                         sizeof(one_key_ack) / sizeof(uint8_t));
}

void McuImpl::pubTmtCltToMid(uint32_t value) {
  rapidjson::Document document;
  document.SetObject();

  rapidjson::Value rcvalue(rapidjson::kObjectType);
  rcvalue.AddMember("data", value, document.GetAllocator());
  
  document.AddMember("remoteControl", rcvalue, document.GetAllocator());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer); 

  std_msgs::String msg;
  msg.data = std::string(buffer.GetString());
  to_middleware_pub_.publish(msg);

  HJ_INFO("pub result:%s\n", buffer.GetString());
}

void McuImpl::pubDirtBoxToMid(uint8_t state) {
  rapidjson::Document document;
  document.SetObject();

  rapidjson::Value stateV(rapidjson::kObjectType);
  stateV.AddMember("state", state, document.GetAllocator());
  
  document.AddMember("dirtybox", stateV, document.GetAllocator());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer); 

  std_msgs::String msg;
  msg.data = std::string(buffer.GetString());
  to_middleware_pub_.publish(msg);

  //HJ_INFO("pub result:%s\n", msg.data.c_str());
}

void McuImpl::mototCurPub(const hj_interface::ElecMotorCur& msg) {
  motot_cur_pub_.publish(msg);
}

void McuImpl::pubPumpMotorSpeed(uint16_t speedl, uint16_t speedr) {
    hj_interface::PumpMotorSpeed msg;
    msg.timestamp = ros::Time::now();
    msg.speed_l = speedl;
    msg.speed_r = speedr;
    pump_motor_speed_pub_.publish(msg);
}

void McuImpl::pubFanMotorSpeed(uint16_t speed) {
    hj_interface::FanMotorSpeed msg;
    msg.timestamp = ros::Time::now();
    msg.speed = speed;
    fan_motor_speed_pub_.publish(msg);
}

void McuImpl::setHealthError(uint32_t code, uint8_t status) {
    hj_interface::HealthCheckCode srv_msg;

    srv_msg.request.code_val = code;
    srv_msg.request.status = status;
    hj_bf::HjPushSrv(srv_msg);
}

void McuImpl::selfHealthCheckAck(health_type type) {
    switch (type) {
        case MOTOR:
            motor_health_ack_ = 1;
        break;

        case IMU:
            imu_health_ack_ = 1;
        break;

        case BMS:
            bms_health_ack_ = 1;
        break;

        case LORA:
           lora_health_ack_ = 1; 
        break;

        default:
            return;
    }

    static uint8_t self_check_ack[] = {0xA5, 0x0,  0x0, 0x0,
                                0xff, 0xff, 0xff, 0xff,
                                0x04, 0x0,
                                0xff, 0xff, 0xff};
                                
    self_check_ack[4] = motor_health_ack_;
    self_check_ack[5] = imu_health_ack_;
    self_check_ack[6] = bms_health_ack_;
    self_check_ack[7] = lora_health_ack_;

    uartDataHandler_->writePackage2Uart(self_check_ack, sizeof(self_check_ack) / sizeof(uint8_t));

    if (motor_health_ack_ == 1 &&
        imu_health_ack_ == 1 &&
        bms_health_ack_ == 1 &&
        lora_health_ack_ == 1) {
        
        hj_interface::HealthCheckCode srv_msg;
        srv_msg.request.code_val = 2999;  // 自检完成标志
        srv_msg.request.status = 0;
        hj_bf::HjPushSrv(srv_msg);
    }
}

void McuImpl::stopGetVerTimer() {
    getMcuVerTmr_.stop();
}

void McuImpl::startMcuTimeSync() {
    rtcSyncTmr_.start();
}

void McuImpl::stopMcuTimeSync() {
    rtcSyncTmr_.stop();
}

void McuImpl::writePubMcuVersion(Mcu_Ver_msg* msg) {
#if HJ_T1pro
     bool is_t1pro = true;
#else
     bool is_t1pro = false;
#endif

  if (g_machine_version == "P4" || is_t1pro) {
    static int ack_count = 0;
    static std::string mcu_ver = "";
    static std::string led_ver = "";
    if (ack_count == 0) {
      mcu_ver = std::to_string(msg->ver_1) +
                      "." +
                      std::to_string(msg->ver_2) +
                      "." +
                      std::to_string(msg->ver_3);
      getmcuLedVer_.start();
    } else if (ack_count == 1) {
      getmcuLedVer_.stop();
      led_ver = std::to_string(msg->ver_1) +
                      "." +
                      std::to_string(msg->ver_2) +
                      "." +
                      std::to_string(msg->ver_3);
      std::ofstream file("/tmp/mcuVer.json", std::ios::out | std::ios::trunc);
      rapidjson::Document doc;
      doc.SetObject();
      doc.AddMember("base_ver", rapidjson::Value(mcu_ver.data(), doc.GetAllocator()).Move(), doc.GetAllocator());
      doc.AddMember("led_ver", rapidjson::Value(led_ver.data(), doc.GetAllocator()).Move(), doc.GetAllocator());

      rapidjson::StringBuffer buffer;
      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
      doc.Accept(writer);

      if (file.is_open()) {
          file << buffer.GetString();
          file.close();
          HJ_INFO("write mcu version [%s] led version[%s]complete\n", mcu_ver.c_str(), led_ver.c_str());
      } else {
          HJ_ERROR("open mcuVer.json fail\n");
      }

      std_msgs::String pubdata;
      pubdata.data = buffer.GetString();
      HJ_INFO("pub mcu version [%s]\n", pubdata.data.c_str());
      mcuVer_pub_.publish(pubdata);
    }
    ack_count++;
  } else if (g_machine_version == "P3.5") {
    static std::string mcu_ver = "";
    static std::string led_ver = "";
    mcu_ver = std::to_string(msg->ver_1) +
                    "." +
                    std::to_string(msg->ver_2) +
                    "." +
                    std::to_string(msg->ver_3);
    led_ver = std::to_string(msg->ver_1) +
                    "." +
                    std::to_string(msg->ver_2) +
                    "." +
                    std::to_string(msg->ver_3);
    std::ofstream file("/tmp/mcuVer.json", std::ios::out | std::ios::trunc);
    rapidjson::Document doc;
    doc.SetObject();
    doc.AddMember("base_ver", rapidjson::Value(mcu_ver.data(), doc.GetAllocator()).Move(), doc.GetAllocator());
    doc.AddMember("led_ver", rapidjson::Value(led_ver.data(), doc.GetAllocator()).Move(), doc.GetAllocator());

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    if (file.is_open()) {
        file << buffer.GetString();
        file.close();
        HJ_INFO("write mcu version [%s] led version[%s]complete\n", mcu_ver.c_str(), led_ver.c_str());
    } else {
        HJ_ERROR("open mcuVer.json fail\n");
    }

    std_msgs::String pubdata;
    pubdata.data = buffer.GetString();
    HJ_INFO("pub mcu version [%s]\n", pubdata.data.c_str());
    mcuVer_pub_.publish(pubdata);
  }
}

void McuImpl::getMcuVerTimerCb(const hj_bf::HJTimerEvent&)
{
    static uint8_t getMcuVer[]= {0xF0, 0x0, 0x0, 0x0,
                                 0x00,
                                 0x01, 0x00,
                                 0xff, 0xff, 0xff};

    uartDataHandler_->writePackage2Uart(getMcuVer,
                                         sizeof(getMcuVer) / sizeof(uint8_t));
}

void McuImpl::getMcuLedVerTimerCb(const hj_bf::HJTimerEvent&) {
  static uint8_t getMcuVer1[]= {0xF0, 0x0, 0x0, 0x0,
                                 0x01,
                                 0x01, 0x00,
                                 0xff, 0xff, 0xff};

  uartDataHandler_->writePackage2Uart(getMcuVer1,
                                        sizeof(getMcuVer1) / sizeof(uint8_t));
}

void McuImpl::rtcTimeSyncTimerCb(const hj_bf::HJTimerEvent&) {
    time_t currentTime = 0;
    ::time(&currentTime);

    struct tm* localTime = localtime(&currentTime);
    uint16_t year = localTime->tm_year + 1900;
    uint16_t month = localTime->tm_mon + 1;
    uint16_t day = localTime->tm_mday;
    uint16_t hour = localTime->tm_hour;
    uint16_t minute = localTime->tm_min;
    uint16_t second = localTime->tm_sec;

    static uint8_t rtc_set[] = {0x74, 0x0, 0x0, 0x0,  
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //year, mon, date
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //hour, min, sec, week
                             0x0D, 0x0, 
                             0xff, 0xff, 0xff};
    
    memcpy(rtc_set+4, &year, sizeof(uint16_t));
    memcpy(rtc_set+6, &month, sizeof(uint16_t));
    memcpy(rtc_set+8, &day, sizeof(uint16_t));
    memcpy(rtc_set+10, &hour, sizeof(uint16_t));
    memcpy(rtc_set+12, &minute, sizeof(uint16_t));
    memcpy(rtc_set+14, &second, sizeof(uint16_t));

    uartDataHandler_->writePackage2Uart(rtc_set, sizeof(rtc_set));
}

void McuImpl::syncRtcWithFlag(const hj_bf::HJTimerEvent&) {
    static int cnt = 0;
    if (cnt >= 150) { //3 min
        HJ_INFO("rtc time sync give up!\n");
        syncRtcWithFlagTmr_.stop();
        return;
    }

    boost::filesystem::path file("/tmp/ntptime.mark");
    if (boost::filesystem::exists(file)) {
        //Network has sync time before, sync rtc
        HJ_INFO("need rtc time sync!\n");
        syncRtcWithFlagTmr_.stop();
        this->startMcuTimeSync();
    } else {
        ++cnt;
    }
}

void McuImpl::airbagCb(const hj_interface::AirBag::ConstPtr& msg) {
  static std::vector<uint8_t> air_bag(airbagCtl_.dataLen(), 0xff);
  uint8_t airbag_status = msg->airbag_ctl;
  uint16_t airbag_time = msg->airbag_time;
  
  air_bag[0] = airbag_status;
  memcpy(air_bag.data() + 1, &airbag_time, sizeof(uint16_t));
  airbagCtl_.pushData(air_bag);
}

void McuImpl::steerPumpCb(const hj_interface::SteerAndPump::ConstPtr& msg) {
  static uint8_t pump_one[] = {0x32, 0x0, 0x0, 0x0, 
                                0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                0x06, 0x0, 
                                0xff, 0xff, 0xff};

  static uint8_t turn_motor_two[] = {0x30, 0x0, 0x0, 0x0, 
                                0xff, 0xff, 0xff, 0xff, 0xff, 
                                0x05, 0x0, 
                                0xff, 0xff, 0xff};

  uint8_t pump_status = msg->pump_ctl;
  uint8_t pump_speed = msg->pump_speed;
  memcpy(pump_one + 4, &pump_status, sizeof(uint8_t));
  memcpy(pump_one + 5, &pump_speed, sizeof(uint8_t));
  uartDataHandler_->writePackage2Uart(pump_one, sizeof(pump_one) / sizeof(uint8_t));

  uint8_t turn_motor_status = msg->turn_motor_ctl;
  int16_t turn_motor_l = msg->turn_motor_l;
  int16_t turn_motor_r = msg->turn_motor_r;
  
  memcpy(turn_motor_two + 4, &turn_motor_status, sizeof(uint8_t));
  memcpy(turn_motor_two + 5, &turn_motor_l, sizeof(int16_t));
  memcpy(turn_motor_two + 7, &turn_motor_r, sizeof(int16_t));
  uartDataHandler_->writePackage2Uart(turn_motor_two,
                                         sizeof(turn_motor_two) / sizeof(uint8_t));
}

void McuImpl::pumpSteerCb(const hj_interface::PumpAndSteer::ConstPtr& msg)
{
  static std::vector<uint8_t> pump_two(pumpCtl_.dataLen(), 0xff);
  static std::vector<uint8_t> turn_motor_one(turnCtl_.dataLen(), 0xff);

  uint8_t pump_ctl = msg->pump_ctl;
  if (pump_ctl != 255) {
    uint8_t pump_speed_l = msg->pump_speed_l;
    uint8_t pump_speed_r = msg->pump_speed_r;
    pump_two[0] = pump_ctl;
    pump_two[1] = pump_speed_l;
    pump_two[2] = pump_ctl;
    pump_two[3] = pump_speed_r;
    pump_two[4] = 1;
    pumpCtl_.pushData(pump_two);  
  }

  uint8_t turn_motor_status = msg->turn_motor_ctl;
  if (turn_motor_status != 255) {
    int16_t turn_angle = msg->turn_motor;
    turn_motor_one[0] = turn_motor_status;
    memcpy(turn_motor_one.data() + 1, &turn_angle, sizeof(int16_t));

    turnCtl_.pushData(turn_motor_one);
  }
}

void McuImpl::turbinMotorCb(const hj_interface::TurbineMotor::ConstPtr& msg) {
  static std::vector<uint8_t> turbin(turbinCtl_.dataLen(), 0xff);
 
  //HJ_INFO("turbin ctl: %d, %d", msg->motor_ctl, msg->motor);

  turbin[0] = msg->motor_ctl;
  turbin[1] = msg->motor;
  turbin[2] = 1;
  
  turbinCtl_.pushData(turbin);
}

void McuImpl::imuWorkModeCb(const hj_interface::ImuWorkModel::ConstPtr& msg) {
  static std::vector<uint8_t> imucal(imuCalCtl_.dataLen(), 0xff);
  imucal[0] = msg->type;
  imuCalCtl_.pushData(imucal);
}

#ifdef X6
void McuImpl::x6pumpCb(const hj_interface::X6Pump::ConstPtr& msg) {
  static uint8_t x6_pump_data[] = {0x32, 0x0, 0x0, 0x0, 
                                   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                                   0x06, 0x0, 
                                   0xff, 0xff, 0xff};
  
  *(x6_pump_data+4) = msg->pump_ctl_l;
  *(x6_pump_data+5) = msg->pump_speed_l;
  *(x6_pump_data+6) = msg->pump_ctl_r;
  *(x6_pump_data+7) = msg->pump_speed_r;

  uartDataHandler_->writePackage2Uart(x6_pump_data,
                                         sizeof(x6_pump_data) / sizeof(uint8_t));
}
#endif

void McuImpl::flipCoverCb(const hj_interface::FlipCover::ConstPtr& msg) {
  static std::vector<uint8_t> flip_cover_data(flipCtl_.dataLen(), 0xff);
  
  uint16_t flip_cover_angle = msg->flip_cover;

  memcpy(flip_cover_data.data(), &flip_cover_angle, sizeof(uint16_t));

  flipCtl_.pushData(flip_cover_data);
}

void McuImpl::mcuCtlFromMiddleCb(const std_msgs::String::ConstPtr& msg) {
  HJ_INFO("mcu ctl receive:\n%s\n", msg->data.c_str());
  rapidjson::Document document;
  if (document.Parse(msg->data.data()).HasParseError()) {
    HJ_ERROR("parse mcuctl fail:\n%s\n", msg->data.c_str());
    return;
  }

  if (document.HasMember("BTN") && document["BTN"].IsObject()) {
    const rapidjson::Value& btn = document["BTN"].GetObject();
    dealButtonCmdJsonFromMid(btn);
  } else if (document.HasMember("remoteControl") && document["remoteControl"].IsObject()) {
    const rapidjson::Value& rmt = document["remoteControl"].GetObject();
    dealRemoteCtlJsonFromMid(rmt);
  } else if (document.HasMember("lightStrip") && document["lightStrip"].IsObject()) {
    const rapidjson::Value& light = document["lightStrip"].GetObject();
    dealLightCtlJsonFromMid(light);
  } else if (document.HasMember("lightPearl") && document["lightPearl"].IsArray()) {
    const rapidjson::Value& light = document["lightPearl"].GetArray();
    dealLightPearlJsonFromMid(light);
  }

}

void McuImpl::dealButtonCmdJsonFromMid(const rapidjson::Value& buttonjson) {
/*    
  static uint8_t light_ctl[] = {0x91, 0x0, 0x0, 0x0,  
                                0xff, 0xff, 0xff,
                                0x03, 0x0, 
                                0xff, 0xff, 0xff};
*/

  static std::vector<uint8_t> button_power_cmd(powerCtl_.dataLen(), 0xff);

  if (!buttonjson.HasMember("pushTime") || !buttonjson["pushTime"].IsInt()) {
    return;
  }

  int type = buttonjson["pushTime"].GetInt();

  switch(type) {
    case 1:
      {
/*
        if (!buttonjson.HasMember("data") || !buttonjson["data"].IsInt()) {
          return;
        }

        int data = 0;
        data = buttonjson["data"].GetInt();
        uint8_t ctl_data[3] = {0x01, 0x01, 0xff};
        switch(data) {
          case 1:
            ctl_data[2] = 0x01;
            break;
          case 2:
            ctl_data[2] = 0x02;
            break;
          case 3:
            ctl_data[2] = 0x03;
            break;
          default:
            HJ_ERROR("unknown type:%d\n", type);
            return;
        }

        memcpy(light_ctl+4, ctl_data, sizeof(ctl_data)/sizeof(uint8_t));
        uartDataHandler_->writePackage2Uart(light_ctl, sizeof(light_ctl)/sizeof(uint8_t));
*/
      }
    break;

    case 2:
      {
        button_power_cmd[0] = 1;
        powerCtl_.pushData(button_power_cmd);
      }
    break;

    case 3:
      {
        button_power_cmd[0] = 2;
        powerCtl_.pushData(button_power_cmd);
      }
    break;

    case 4:
      {
        button_power_cmd[0] = 3;
        powerCtl_.pushData(button_power_cmd);
      }
    break;

    default:
      HJ_ERROR("unknown type:%d\n", type);
  }
}

void McuImpl::dealRemoteCtlJsonFromMid(const rapidjson::Value& rmtctljson) {
  if (!rmtctljson.HasMember("mode") || !rmtctljson["mode"].IsInt()) {
    return;
  }
  
  int mode = 0;
  mode = rmtctljson["mode"].GetInt();

  switch (mode)
  {
  case 1:
    {
      if (rmtctljson.HasMember("left") && rmtctljson.HasMember("right")) {
        if (rmtctljson["left"].IsInt() && rmtctljson["right"].IsInt()) {
          int left = 0; 
          int right = 0;
          left = rmtctljson["left"].GetInt();
          right = rmtctljson["right"].GetInt();
          uint8_t send_speed[] = {0x10, 0x0,  0x0,  0x0,  
                                  0xff, 0xff, 0xff, 0xff, 
                                  0xff, 0xff, 0xff, 0xff,
                                  0x08, 0x0,  0xff, 0xff, 0xff};
          memcpy(send_speed + 4, &left, sizeof(int32_t));
          memcpy(send_speed + 8, &right, sizeof(int32_t));
          uartDataHandler_->writePackage2Uart(send_speed,
                                         sizeof(send_speed) / sizeof(uint8_t));
        }
      }
      /*
      if (rmtctljson.HasMember("pumpA") && rmtctljson.HasMember("pumpB")) {
        if (rmtctljson["pumpA"].IsInt() && rmtctljson["pumpB"].IsInt()) {
          int pumpA = rmtctljson["pumpA"].GetInt();
          int pumpB = rmtctljson["pumpB"].GetInt();
          uint8_t pump[] = {0x32, 0x0, 0x0, 0x0, 
                            0xff, 0xff, 0xff, 0xff, 0xff,
                            0x05, 0x0, 
                            0xff, 0xff, 0xff};

          *(pump + 4) = 1;
          *(pump + 5) = pumpA;
          *(pump + 6) = 1;
          *(pump + 7) = pumpB;
          *(pump + 8) = 1;
          uartDataHandler_->writePackage2Uart(pump, sizeof(pump) / sizeof(uint8_t));
        }
      }
      */
    }
  break;
  
  case 32:
    {
      if (rmtctljson.HasMember("airbagCtl") && rmtctljson.HasMember("airbagTime")) {
        if (rmtctljson["airbagCtl"].IsInt() && rmtctljson["airbagTime"].IsInt()) {
          int ctl = 0;
          int time = 0;
          ctl = rmtctljson["airbagCtl"].GetInt();
          time = rmtctljson["airbagTime"].GetInt();
          uint8_t air_bag[] = {0x34, 0x0,  0x0, 0x0,  
                               0xff, 0xff, 0xff, 
                               0x03, 0x0, 
                               0xff, 0xff, 0xff};

          memcpy(air_bag + 4, &ctl, sizeof(uint8_t));
          memcpy(air_bag + 5, &time, sizeof(uint16_t));
          uartDataHandler_->writePackage2Uart(air_bag,
                                                sizeof(air_bag) / sizeof(uint8_t));
        }
      }
    }
  break;

  default:
    break;
  }
}

void McuImpl::dealLightCtlJsonFromMid(const rapidjson::Value& lightCtlJson) {
    static std::vector<uint8_t> light(lightStripCtl_.dataLen(), 0xff);

    if ((lightCtlJson.HasMember("status") && lightCtlJson["status"].IsInt()) &&
        (lightCtlJson.HasMember("color") && lightCtlJson["color"].IsInt())) {
        
        uint8_t status = 0;
        uint8_t color = 0;
        status = lightCtlJson["status"].GetInt();
        color = lightCtlJson["color"].GetInt();

        light[0] = status;
        light[1] = color;
        
        lightStripCtl_.pushData(light);
        //uartDataHandler_->writePackage2Uart(light, sizeof(light) / sizeof(uint8_t));
    }
}

void McuImpl::dealLightPearlJsonFromMid(const rapidjson::Value& lightCtlJson) {
    static std::vector<uint8_t> lightpearl(lightPearlCtl_.dataLen(), 0xff);

    if (!lightCtlJson.IsArray()) {
        return;
    }

    if (lightCtlJson.Size() != lightPearlCtl_.dataLen()) {
        HJ_ERROR("wrong light ctl size:%d\n", lightCtlJson.Size());
        return;
    }

    for (rapidjson::SizeType i = 0; i < lightCtlJson.Size(); ++i) {
        lightpearl[i] = lightCtlJson[i].GetInt();
    }

    HJ_INFO("light pearl ctl\n");

    lightPearlCtl_.pushData(lightpearl);
}

void McuImpl::rtcTimeSyncCb(const std_msgs::String::ConstPtr& msg) {
    HJ_INFO("rtc time sync trigger:%s\n", msg->data.c_str());
    this->startMcuTimeSync();
}

/*上游算法的控制*/
void McuImpl::motor_set_chatterCallback(const hj_interface::Nav::ConstPtr &msg) {
  // HJ_INFO("motor_motor receive : left_msg = %f , right_msg = %f ", msg->left_msg, msg->right_msg);

  static std::vector<uint8_t> motor(motorCtl_.dataLen(), 0xff);
  int32_t lw_speed = msg->left_msg;
  int32_t rw_speed = msg->right_msg;
  memcpy(motor.data(), &lw_speed, sizeof(int32_t));
  memcpy(motor.data() + 4, &rw_speed, sizeof(int32_t));
  motorCtl_.pushData(motor);
}

void McuImpl::beatMcu() {
  static uint8_t heart_buf[] = {0x01, 0x0,  0x0,  0x0,  0xff, 0xe8, 0x03,
                                0x00, 0x00, 0x05, 0x00, 0xff, 0xff, 0xff};
  uartDataHandler_->writePackage2Uart(heart_buf, sizeof(heart_buf));
}

void McuImpl::heartBeatTimerCb(const hj_bf::HJTimerEvent &) {
  beatMcu();
}

Mcu::~Mcu() {
  delete mcuImpl_;
}
  
Mcu::Mcu(const rapidjson::Value &json_conf)
    : hj_bf::Function(json_conf), mcuImpl_(nullptr) {
  bool keylog = false;
  if (json_conf.HasMember("enable_keylog") && json_conf["enable_keylog"].IsInt()) {
    int enable = 0;
    enable = json_conf["enable_keylog"].GetInt();
    if (enable == 1) {
        keylog = true;
    }
  }

  if (json_conf.HasMember("machine_version") && json_conf["machine_version"].IsString()) {
    g_machine_version = json_conf["machine_version"].GetString();
  }

  if (json_conf.HasMember("mcu_para") && json_conf["mcu_para"].IsObject()) {
    auto mcupara = json_conf["mcu_para"].GetObject();
    std::string port = mcupara["port"].GetString();
    int baud = mcupara["baudRate"].GetInt();
    int flow_ctrl = mcupara["flowCtl"].GetInt();
    int databits = mcupara["dataBits"].GetInt();
    int stopbits = mcupara["stopBits"].GetInt();
    std::string parity = mcupara["parity"].GetString();
  
    mcuImpl_ = new McuImpl(port);

    assert(mcuImpl_);
    /*TODO:run失败后需要有重启机制*/
    mcuImpl_->run(baud, flow_ctrl, databits, stopbits, parity[0], keylog);
  } else {
    mcuImpl_ = new McuImpl("/dev/ttyS4");
    assert(mcuImpl_);
    /*TODO:run失败后需要有重启机制*/
    mcuImpl_->run(460800, 0, 8, 1, 'N', keylog);
  }

  HJ_INFO("beat mcu init success");

}

} // end namespace collect_node_mcu
