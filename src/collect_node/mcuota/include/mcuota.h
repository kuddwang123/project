// @file mcuota.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: yzupl1995@163.com (panlou, 2024-02-19)
#ifndef INCLUDE_MCUOTA_H  // your macro
#define INCLUDE_MCUOTA_H
#include "function_factory.h"
#include "node_factory.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "uart.h"
#include <thread>
#include <mutex>
#include "hj_interface/OtaUpgradeData.h"

namespace collect_node_mcu_ota {  // your namespace

using namespace hj_bf;

#define MCU_MAIN_BOARD 0
#define MCU_LAMP_BOARD 1
#define MCU_ANGO_BOARD 2
#define OTA_ACK_ID 0X701
#define OTA_MCU_DATA_ID 0x703
#define OTA_LAMP_DATA_ID 0x705
#define OTA_ANGO_DATA_ID 0x707
#define MCU_REQ_FIRMINFO_CMD 0x01
#define SOC_SEND_FIRMINFO_CMD 0x02
#define MCU_REQ_BINDATA_CMD 0x05
#define SOC_SEND_BINDATA_CMD 0x06
#define MCU_OTA_RSTCHECK_CMD 0x08
#define SOC_QUIT_MCUOTA_CMD 0x09

#define MAX_PACKAGE_SIZE 64
#define FRAMEHEAD 0xAA
#define FRAMETAIL 0x55
#define FRAMECTRL 0xA5

enum ota_type {
  CHARGER = 1,
  LAMPBOARD = 2,
  MCU = 3,
  ANGO = 5
};

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
  }while(0)                                              \


class UartOtaDataHandler;

class State {
  public:
    State(int fd, uint32_t timeout, UartOtaDataHandler*);
    virtual ~State();  
    virtual bool handle() = 0;
    virtual void reset();
    void setMcuOtaId(uint8_t id);
    void setNextSuccState(State*);
    void setNextFailState(State*);
    State* nextSuccState();
    State* nextFailState();
    bool getResult() const {return result_;}
    const std::string& getRstMsg() {return rst_msg_;}

  protected:
    int uartPkgFd_;
    uint32_t mcuTimeOutSec_;
    uint8_t mcuid_;
    uint32_t sendDataToMcuId_;
    uint32_t dataFromMcuId_;
    UartOtaDataHandler* uartHandler_;
    State* nextSuccState_;
    State* nextFailState_;
    bool result_;
    std::string rst_msg_;
    hj_bf::HJTimer writeTmr_;
    #define STATERET(v,m)  writeTmr_.stop(); result_=v; rst_msg_=m; return result_;
};

class StartOta: public State {
  public:
    StartOta(int fd, UartOtaDataHandler*);
    ~StartOta() {};
    bool handle() override;
  
  private:
    void writeUartTmrCb(const hj_bf::HJTimerEvent&);
};

class FirmwareInfo: public State {
  public:
    FirmwareInfo(int fd, UartOtaDataHandler*);
    ~FirmwareInfo() {};
    void init(uint32_t size, const std::string& ver);
    bool handle() override;

  private:
    uint8_t verCount_;
    uint32_t size_;
    std::string ver_;
    bool writeFirmDone_;
    std::vector<uint16_t> parsedVerNum_;

  private:
    bool parseVer(const std::string&);
    void writeFirmTmrCb(const hj_bf::HJTimerEvent&);
};

class SendBin: public State {
  public:
    SendBin(int fd, UartOtaDataHandler*);
    ~SendBin() {};
    void init(const std::vector<uint8_t>& data);
    bool handle() override;
    void reset() override;

  private:
    uint16_t index_;
    uint8_t retryCnt_;
    std::vector<uint8_t> data_;
    const int MAX_DATA_CNT = 1024;
  
  private:
    uint16_t sendBin2uart(uint16_t index, uint32_t offset);
    void sendBinTmrCb(const hj_bf::HJTimerEvent&) {};
};

/*ota串口数据的处理*/
class UartOtaDataHandler {
  public:
    UartOtaDataHandler();
    ~UartOtaDataHandler();
    bool writePackage2Uart(uint8_t *buf, uint32_t len);
    bool analyzePackage();
    void initialize(const std::string&, int, int, int, int, int, int);
    void reset();

  private:
    uint8_t uart_lastByte_;
    bool uart_beginFlag_;
    bool uart_ctrlFlag_;
    uint8_t uart_revOffset_;
    uint8_t checkSum_;
    uint8_t rxPackageDataCount_;
    uint8_t u_recvBuf_[MAX_PACKAGE_SIZE];
    uint8_t m_pRxBuf[MAX_PACKAGE_SIZE];
    int writefd_;
    Uart* uart_;
};


/*mcu ota调度*/
class McuOtaRun {
typedef boost::function<void(ota_type, bool, const std::string&)> pubMcuOtaRstFunc;
  public:
    McuOtaRun(std::string uart, int, int, int, int, int);
    ~McuOtaRun();
    bool init(uint8_t type, const std::string& vernum, const std::string& binfile);
    void otaRun();
    void joinThreads();
    bool isOtaRun() const {return isOtaRun_;}
    bool getMcuOtaResult() const {return isOtaSucc_;}
    void setMcuOtaResultPubFunc(const pubMcuOtaRstFunc& cb) { pubMcuOtaRstCb_ = cb; }
    bool resetMcu();
    ota_type getOtaType() {return otaType_;}

  private:
    std::string uartdev_;
    std::map<std::string, State*> states_;
    State* curState_;
    UartOtaDataHandler* uartHandler_;
    bool isOtaRun_;
    bool isOtaSucc_;
    uint8_t mcuId_;
    ota_type otaType_;
    std::string verNum_;
    std::string binFile_;
    std::vector<uint8_t> binData_;
    std::thread runMachineState_;
    std::thread runUartHandler_;
    int rwFd_[2];
    pubMcuOtaRstFunc pubMcuOtaRstCb_;
    hj_bf::HJTimer restTmr_;

  private:
    void runMachineState();
    void runUartHandler();
    bool loadBinFile();
    void uartInit(std::string, int, int, int, int, int, int);
    void stateMachineConstruct();
    void stateMachineInit(uint8_t id);
    void resetTimerCb(const hj_bf::HJTimerEvent&);
};

class McuOta : public hj_bf::Function {
  public:
    explicit McuOta(const rapidjson::Value& json_conf);
    ~McuOta();
  
  private:
    McuOtaRun* mcuOtaRun_;
    hj_bf::HJSubscriber mcuOtaCtl_;
    hj_bf::HJSubscriber mcuOta_;
    hj_bf::HJPublisher mcuctl_pub_;
    hj_bf::HJPublisher mcuOtaRstPub_;

  private:
    void mcuOtaCtlCallback(const hj_interface::OtaUpgradeData::ConstPtr&);
    void mcuOtaReadyCallBack(const std_msgs::Bool::ConstPtr&);
    void otaThreadJoinFunc();
    void pubMcuOtaResult(ota_type, bool, const std::string&);
    void mcuOtaReadyTimerCallBack(const hj_bf::HJTimerEvent&);

  private:
    std::thread otaThreadJoin_;
    std::timed_mutex tmtx_;
    hj_bf::HJTimer mcuOtaReadyTmr_;
};

}  // namespace collect_node_mcu_ota

#endif
