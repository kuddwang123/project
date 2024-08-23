// @file mcu.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-26)
#ifndef INCLUDE_MCU_H  // your macro
#define INCLUDE_MCU_H
#include <deque>
#include "function_factory.h"
#include "node_factory.h"
#include "hj_interface/Bat.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Nav.h"
#include "hj_interface/Kbd.h"
#include "hj_interface/AirBag.h"
#include "hj_interface/SteerAndPump.h"
#include "hj_interface/FlipCover.h"
#include "hj_interface/PumpAndSteer.h"
#include "hj_interface/TurbineMotor.h"
#include "hj_interface/ElecMotorCur.h"
#include "hj_interface/ImuWorkModel.h"
#include "hj_interface/HealthCheckCode.h"
#include "hj_interface/HealthCheckCodeRequest.h"
#include "hj_interface/HealthCheckCodeResponse.h"
#include "hj_interface/TempHumidity.h"
#include "hj_interface/DownRay.h"
#ifdef HJ_T1pro
#include "hj_interface/LeftTof.h"
#include "hj_interface/Turbidity.h"
#include "hj_interface/DustPlugDetection.h"
#else
#include "hj_interface/DownLeft.h"
#endif
#include "status_code.h"

#ifdef X6
#include "hj_interface/X6Pump.h"
#endif
#include "uart.h"
#include <thread>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#define DEBUG 1
#ifdef DEBUG
#include "hj_interface/Atime.h"
#endif

#define FRAMEHEAD 0xAA
#define FRAMETAIL 0x55
#define FRAMECTRL 0xA5

#define MAX_PACKAGE_SIZE 64

#define MOTOR_MSG_ACK 0x11
#define MOTOR_MSG_ID 0x12
#define IMU_MSG_ID 0x13
#define IMU_CALI_ID 0x14
#define IMU_CALI_ACK 0x15
#define BAT_MSG_ID 0x20
#define DOWNWARD_SENSOR_ID 0x21
#define OUTWATER_DETEC_ID 0x22
#define TEMP_HUMIDITY_ID 0x23
#define RETURN_CHARGER_ID 0x24
#define DIRTYBOX_DETECT_ID 0x25
#define PUMP_MOTOR_SPEED 0x26
#define FAN_MOTOR_SPEED 0x27
#define TURN_MOTOR_ACK 0x31
#define PUMP_MSG_ACK 0x33
#define AIRBAG_CTL_ACK 0x35
#define DTOF_SENSOR_ID 0x28
#define FLIP_COVER_ACK 0x39
#define TURBIN_CTL_ACK 0x37
#define LIGHT_STRIP_ACK 0x93
#define LIGHT_PEARL_ACK 0x95
#ifdef HJ_T1pro
#define TURBIDITY_SENSOR_ID 0x29
#define DUST_PLUG_DETECTION 0x2A
#endif

#define REMOTE_CONTROL_ID 0x8A
#define MCU_KEY_LOG 0x60
#define MCU_FAIL_CODE 0xA2
#define MCU_SELF_CHECK 0xA4
#define MCU_SELF_CHECK_ACK 0xA5

#define LORA_TRANSPORT_ID 0x82
#define MCU_ONEKEY_TRIGGER 0x90
#define MCU_E_MOTOR 0xA3

#define MCU_VER_ACK 0xF1
#define MCU_RESET_ACK 0x51

#define RTC_TIMESYNC_ACK 0x75
namespace collect_node_mcu {  // your namespace

#define UINT32_TO_BUF_LITTLE(data, buf)                   \
  do {                                                    \
    *(buf) = static_cast<uint8_t>(data & 0xFF);           \
    *(buf+1) = static_cast<uint8_t>((data >> 8) & 0xFF);  \
    *(buf+2) = static_cast<uint8_t>((data >> 16) & 0xFF); \
    *(buf+3) = static_cast<uint8_t>((data >> 24) & 0xFF); \
  }while(0)                                              \

#define UINT16_TO_BUF_LITTLE(data, buf)                   \
  do {                                                    \
    *(buf) = static_cast<uint8_t>(data & 0xFF);           \
    *(buf+1) = static_cast<uint8_t>((data >> 8) & 0xFF);  \
  }while(0) 

typedef struct {
  u_int32_t id;      /* message ID              */
  int32_t lw_speed;  //  mm/s
  int32_t rw_speed;  //  mm/s
  u_int32_t time_ms;
  int8_t len;
  int8_t ch;
  int8_t format;
  int8_t type;
} __attribute__((packed)) Motor_msg;

typedef struct {
  u_int32_t id;      /* message ID              */
  int32_t lw_speed;  //  mm/s
  int32_t rw_speed;  //  mm/s
  int8_t len;
  int8_t ch;
  int8_t format;
  int8_t type;
} __attribute__((packed)) Motor_msg_ack;

typedef struct {
  u_int32_t id;
  u_int16_t pump_a;
  u_int16_t pump_b;
  int8_t len;
  int8_t ch;
  int8_t format;
  int8_t type;
} __attribute__((packed)) Pump_msg_ack;

typedef struct {
  u_int32_t id;                 /* message ID              */
  int16_t  yaw;
  int16_t  pitch;
  int16_t  roll;
  int16_t  gyrox;
  int16_t  gyroy;
  int16_t  gyroz;
  int16_t  accx;
  int16_t  accy;
  int16_t  accz;
  u_int32_t  time_ms;
  uint8_t  flag;
  int16_t  len;
  int8_t  ch;
  int8_t  format;
  int8_t  type;
} __attribute__((packed)) IMU_msg;

typedef struct {
  uint32_t id;                 /* message ID              */
  uint8_t  power;
  int8_t temp1;
  int8_t temp2;
  int8_t temp3;
  uint16_t bat_vol;
  int16_t bat_disch_cur;
  uint16_t ch_vol;
  int16_t charger_ch_cur;
  uint16_t cycle_times;
  uint8_t health_left;
} __attribute__((packed)) BAT_msg;

typedef struct {
  u_int32_t id;
  uint8_t flag;
  uint8_t value;
} __attribute__((packed)) Key_msg;

typedef struct {
  u_int32_t id;
  uint16_t motor_l;
  uint16_t motor_r;
  uint16_t pump_l;
  uint16_t pump_r;
  uint16_t turn;
  uint16_t dirtybox;
  uint16_t flipcover;
  uint16_t airbag;
} __attribute__((packed)) Emotor_Cur_msg;

typedef struct {
  u_int32_t id;
  uint8_t key1;
  uint8_t key2;
} __attribute__((packed)) Lora_header;

typedef struct {
  u_int32_t id;
  uint8_t key1;
  uint8_t key2;
  uint8_t flag;
  uint16_t ultra_lt[5];
  uint16_t ultra_rt[5];
} __attribute__((packed)) Station_Ultra_msg;

typedef struct {
  u_int32_t id; 
  u_int16_t  humidity;
  u_int16_t  temperature;
  int8_t len;
  int8_t ch;
  int8_t format;
  int8_t type;
} __attribute__((packed)) Temp_Humidity;

typedef struct {
  u_int32_t id; 
  u_int16_t  ver_1;
  u_int16_t  ver_2;
  u_int16_t  ver_3;
  u_int8_t   hw_ver;
} __attribute__((packed)) Mcu_Ver_msg;

typedef struct {
  u_int32_t id;
  uint8_t type;
  uint8_t left_wheel_status;       // 左驱动轮状态
  uint8_t rignt_wheel_status;      // 右驱动轮状态
  uint8_t steering_motor_status;   // 转向电机状态
  uint8_t left_water_pump_status;  // 左水泵状态
  uint8_t right_water_pump_status;  // 右水泵状态
  uint8_t airbag_status;            // 气囊电机状态
  uint8_t fan_status;               // 风机状态
  uint8_t flip_cover_motor_status;  // 翻盖电机状态
} ElectricalMachineryStatus;

typedef struct {
  uint32_t  id; 
  uint16_t  speed_l;
  uint16_t  speed_r;
} __attribute__((packed)) Pump_Motor_Speed_msg;

typedef struct {
  uint32_t  id; 
  uint16_t  speed;
} __attribute__((packed)) Fan_Motor_Speed_msg;

typedef struct {
    uint32_t id;
    uint8_t result;
} __attribute__((packed)) Rtc_TimeSync_Ack_msg;

typedef struct {
  uint32_t  id;
  uint16_t  dist_left;
  uint16_t  dist_front;
} __attribute__((packed)) Tof_Sensor_msg;

/*循环队列*/
class Queue {
  public:
    Queue() = delete;
    explicit Queue(unsigned int);
    Queue(const Queue&);
    Queue& operator=(const Queue&);
    
    ~Queue();
    bool isFull();
    bool isEmpty();
    bool enqueue(uint8_t);
    uint8_t dequeue();

  private:
    unsigned int size_;
    int head_;
    int tail_;
    uint8_t* buffer_;
};

enum health_type {
    MOTOR,
    IMU,
    BMS,
    LORA
};

/*串口数据的处理*/
class McuImpl;
class UartDataHandler {
  public:
    UartDataHandler();
    ~UartDataHandler();
    void writePackage2Uart(uint8_t *buf, uint8_t len);
    bool analyzePackage(McuImpl*);
    bool initialize(const std::string&, int, int, int, int, int, bool);
    void reset();
  
  private:
    uint8_t uart_lastByte_;
    bool uart_beginFlag_;
    bool uart_ctrlFlag_;
    uint8_t uart_revOffset_;
    uint8_t checkSum_;
    uint8_t rxPackageDataCount_;
    int time_update_;
    double time_diff_;
    double time_diff_sum_{0.0};
    uint8_t u_recvBuf_[MAX_PACKAGE_SIZE];
    hj_bf::Uart* uart_;
    uint8_t m_pRxBuf[MAX_PACKAGE_SIZE];
    uint64_t motorcnt_;
    uint64_t imucnt_;
    std::string keylogDir_;
    void* keylog_;
};

class SensorCtl {
  struct uartdata {
    std::vector<uint8_t> _data;
    int _maxcnt;
    bool _acked;
    
    uartdata(const std::vector<uint8_t>& data):
        _data(data),
        _maxcnt(5),
        _acked(false) {
        
    }
    ~uartdata() {
        
    };
  };

  public:
    SensorCtl();
    ~SensorCtl();
    void init(uint32_t id, uint16_t len, UartDataHandler* uarthl, 
        const std::string& tmrName, bool needCheckAck = true, bool needCmpLast = true);
    uint16_t dataLen();
    void pushData(const std::vector<uint8_t>&);
    void ack(uint8_t*);

  private:
    uint32_t id_;
    uint16_t len_;
    uint8_t head_[4];
    uint8_t tail_[5];
    bool needCheckAck_;
    bool needCmpLast_;
    uint8_t* uartdata_;
    UartDataHandler* uarthl_;
    std::vector<uint8_t> lastwrit_;
    hj_bf::HJTimer ackTmr_;
    std::deque<uartdata> queue_;
    std::atomic_flag atmoicFlag_;

  private:
    uint16_t sendDataLen();
    void spinLock();
    void spinUnLock();
    void ackTimerCb(const hj_bf::HJTimerEvent&);
    void writeUart(uartdata&);
};

/*mcu执行类，负责处理底板和核心板的交互逻辑*/
class McuImpl {
  public:
    explicit McuImpl(std::string);
    ~McuImpl();
    bool run(int, int, int, int, int, bool keylog);
#ifdef HJ_T1pro
    void TofPub(const hj_interface::LeftTof&);
    void InfraredSensorPub(const hj_interface::DownRay&);
    void TurbidityPub(const hj_interface::Turbidity&);
    void DustPlugPub(const hj_interface::DustPlugDetection&);
#else
    void InfraredSensorPub(const hj_interface::DownLeft&);
    void InfraredSensorPub(const hj_interface::DownRay&);
#endif
    void motorPub(const hj_interface::Encoder&);
    void imuPub(const hj_interface::Imu&);
#ifdef DEBUG
    void motorTimePub(const hj_interface::Atime&);
    void imuTimePub(const hj_interface::Atime&);
#endif
    void tempHumidityPub(const hj_interface::TempHumidity&);
    void batPub(const hj_interface::Bat&);
    void mototCurPub(const hj_interface::ElecMotorCur&);
    void stationUltraPub(Station_Ultra_msg*);
    void writePubMcuVersion(Mcu_Ver_msg*);
    void outwaterPub(uint8_t);
    void onChargerPub(uint8_t);
    void pubFirstImuTime(uint32_t);
    void stopSpeedTimer();
    void stopGetVerTimer();
    void stopImuCaliTimer(uint8_t);
    void startMcuTimeSync();
    void stopMcuTimeSync();
    void setHealthError(uint32_t, uint8_t);
    void selfHealthCheckAck(health_type);
    void pubKeyCtlToMid(uint8_t, uint8_t);
    void pubTmtCltToMid(uint32_t);
    void pubDirtBoxToMid(uint8_t);
    void pubPumpMotorSpeed(uint16_t, uint16_t);
    void pubFanMotorSpeed(uint16_t);
    void pubTimeDiff(double);
    void sensorCtlAck(uint32_t ackid, uint8_t* data);

  private:
    std::string dev_;
    bool isRun_;
    uint8_t motor_health_ack_;
    uint8_t imu_health_ack_;
    uint8_t bms_health_ack_;
    uint8_t lora_health_ack_;
    UartDataHandler* uartDataHandler_;
    hj_bf::HJPublisher motor_pub_;
    hj_bf::HJPublisher imu_pub_;
#ifdef DEBUG
    hj_bf::HJPublisher motor_time_pub_;
    hj_bf::HJPublisher imu_time_pub_;
#endif
    hj_bf::HJPublisher bat_pub_;
    hj_bf::HJPublisher mcuota_pub_;
    hj_bf::HJPublisher motot_cur_pub_;
    hj_bf::HJPublisher outof_water_pub_;
    hj_bf::HJPublisher station_ultra_pub_;
    hj_bf::HJPublisher to_middleware_pub_;
    hj_bf::HJPublisher tempHumidity_pub_;
#if HJ_T1pro
    hj_bf::HJPublisher infrared_sensor_pub_;
    hj_bf::HJPublisher tof_pub_;
    hj_bf::HJPublisher turbidity_pub_;
    hj_bf::HJPublisher dust_plug_pub_;
#else
    hj_bf::HJPublisher infrared_sensor_pub_;
#endif
    hj_bf::HJPublisher mcuVer_pub_;
    hj_bf::HJPublisher ret_charger_pub_;
    hj_bf::HJPublisher pump_motor_speed_pub_;
    hj_bf::HJPublisher fan_motor_speed_pub_;
    hj_bf::HJPublisher first_imu_pub_;
    hj_bf::HJPublisher time_diff_pub_;
    hj_bf::HJSubscriber sub_motor_set_;
    hj_bf::HJSubscriber sub_airbag_;
    hj_bf::HJSubscriber sub_steerpump_;
    hj_bf::HJSubscriber sub_flipcover_;
    hj_bf::HJSubscriber sub_mcuctl_;
    hj_bf::HJSubscriber sub_pumpsteer_;
    hj_bf::HJSubscriber sub_turbin_motor_;
    hj_bf::HJSubscriber sub_imucal_;
    hj_bf::HJSubscriber sub_middleware_;
    hj_bf::HJSubscriber sub_rtcTimeSync_;
  #ifdef X6
    hj_bf::HJSubscriber sub_x6pump_;
  #endif
    hj_bf::HJTimer heartBeat_;
    hj_bf::HJTimer getMcuVerTmr_;
    hj_bf::HJTimer getmcuLedVer_;
    hj_bf::HJTimer rtcSyncTmr_;
    hj_bf::HJTimer syncRtcWithFlagTmr_;
    std::thread dealUartThred_;

    SensorCtl motorCtl_;
    SensorCtl airbagCtl_;
    SensorCtl imuCalCtl_;
    SensorCtl flipCtl_;
    SensorCtl pumpCtl_;
    SensorCtl turnCtl_;
    SensorCtl turbinCtl_;
    SensorCtl powerCtl_;
    SensorCtl lightStripCtl_;
    SensorCtl lightPearlCtl_;

  private:
    void motor_set_chatterCallback(const hj_interface::Nav::ConstPtr&);
    void mcuCtlCallBack(const std_msgs::Bool::ConstPtr&);
    void speedTimerCb(const hj_bf::HJTimerEvent&);
    void imuCaliEnableTimerCb(const hj_bf::HJTimerEvent&);
    void imuCaliDisableTimerCb(const hj_bf::HJTimerEvent&);
    void heartBeatTimerCb(const hj_bf::HJTimerEvent&);
    void getMcuVerTimerCb(const hj_bf::HJTimerEvent&);
    void getMcuLedVerTimerCb(const hj_bf::HJTimerEvent&);
    void rtcTimeSyncTimerCb(const hj_bf::HJTimerEvent&);
    void syncRtcWithFlag(const hj_bf::HJTimerEvent&);
    void airbagCb(const hj_interface::AirBag::ConstPtr&);
    void steerPumpCb(const hj_interface::SteerAndPump::ConstPtr&);
    void flipCoverCb(const hj_interface::FlipCover::ConstPtr&);
    void pumpSteerCb(const hj_interface::PumpAndSteer::ConstPtr&);
    void turbinMotorCb(const hj_interface::TurbineMotor::ConstPtr&);
    void imuWorkModeCb(const hj_interface::ImuWorkModel::ConstPtr&);
    void mcuCtlFromMiddleCb(const std_msgs::String::ConstPtr&);
    void rtcTimeSyncCb(const std_msgs::String::ConstPtr&);
    
  #ifdef X6
    void x6pumpCb(const hj_interface::X6Pump::ConstPtr&);
  #endif
    void initSensorCtl();
    void beatMcu();
    void dealUartLoop(); 

    void dealButtonCmdJsonFromMid(const rapidjson::Value&);
    void dealRemoteCtlJsonFromMid(const rapidjson::Value&);
    void dealLightCtlJsonFromMid(const rapidjson::Value&);
    void dealLightPearlJsonFromMid(const rapidjson::Value&);
    
  private:
    enum {
      PUMP_SWITCH = 0x10,           // pump_status
      PUMP_FLOWRATE = 0x11,         // pump_speed
      LEFT_SPOUT_VEER = 0x12,       // turn_motor_l
      RIGHT_SPOUT_VEER = 0x13,      // turn_motor_r
      SLEF_CLEAN_SWITCH = 0x14,     // fan_status
      SLEF_CLEAN_ROTATION = 0x15,   // not use
      PUMP_DUTY_CYCLE = 0x16,       // pump_speed
      UPPER_SUCTION_SWITCH = 0x17,  // flip_cover_angle
      DIVE_MOTOR = 0x18,            // airbag_status
      INFLACTION_TIME = 0x19,       // airbag_time
      WHEEL = 0x20                  // left_msg right_msg
    };
};



class Mcu : public hj_bf::Function {
  public:
    explicit Mcu(const rapidjson::Value& json_conf);
    ~Mcu();
  
  private:
    McuImpl* mcuImpl_;
};

}  // namespace collect_node_mcu

#endif
