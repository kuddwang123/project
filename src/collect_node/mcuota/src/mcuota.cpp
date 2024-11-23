#include "mcuota.h"
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include "log.h"
#include "hjlog.h"
#include <sstream>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <regex>
#include "hj_interface/OtaUpgradeStatus.h"

HJ_REGISTER_FUNCTION(factory) {
  HJ_INFO("minos register factory, funtion_name:%s", FUNCTION_NAME);
  factory.registerCreater<collect_node_mcu_ota::McuOta>(FUNCTION_NAME);
}

namespace collect_node_mcu_ota {

static void* logger = nullptr;

#ifdef DEBUG
#undef HJ_INFO
#undef HJ_DEBUG
#undef HJ_ERROR

#define HJ_INFO(fmt, ...) HJ_CST_TIME_INFO(logger, fmt, ##__VA_ARGS__);
#define HJ_DEBUG(fmt, ...) HJ_CST_TIME_DEBUG(logger, fmt, ##__VA_ARGS__);
#define HJ_ERROR(fmt, ...) HJ_CST_TIME_ERROR(logger, fmt, ##__VA_ARGS__);
#endif

State::State(int uartfd, uint32_t timeout, UartOtaDataHandler* uartHandler):
    uartPkgFd_(uartfd), mcuTimeOutSec_(timeout), uartHandler_(uartHandler),
    nextSuccState_(nullptr), nextFailState_(nullptr), result_(false) {}

State::~State() {

}

void State::setNextSuccState(State* state) {
    nextSuccState_ = state;
}

void State::setNextFailState(State* state) {
    nextFailState_ = state;
}

State* State::nextSuccState() {
    return nextSuccState_;
}

State* State::nextFailState() {
    return nextFailState_;
}

void State::reset() {
    result_ = false;
}

void State::setMcuOtaId(uint8_t id) 
{
    mcuid_ = id;
    if (mcuid_ == MCU_MAIN_BOARD) {
        sendDataToMcuId_ = 0x702;
        dataFromMcuId_ = 0x703;
    } else if (mcuid_ == MCU_LAMP_BOARD) {
        sendDataToMcuId_ = 0x704;
        dataFromMcuId_ = 0x705;
    }
}

StartOta::StartOta(int fd, UartOtaDataHandler* uartHandler):
    State(fd, 5, uartHandler) {
    writeTmr_ = hj_bf::HJCreateTimer("startota", 100 * 1000, &StartOta::writeUartTmrCb, this, false);
}

bool StartOta::handle()
{
    HJ_INFO("****enter startota****\n");
    HJ_INFO("mcuid:%d\n", mcuid_);
    /*
    uint8_t ota[] = {0x00, 0x07, 0x00, 0x00, 
                     mcuid_, 
                     0x01, 0x00, 
                     0xff, 0xff, 0xff};

    uartHandler_ -> writePackage2Uart(ota, sizeof(ota)/sizeof(uint8_t));
    */
    writeTmr_.start();
    int ret = 0;
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = mcuTimeOutSec_;
    timeout.tv_usec = 0;
    uint8_t buf[MAX_PACKAGE_SIZE] = {0};
    uint32_t id = 0;

    while (timeout.tv_sec > 0) {
        FD_ZERO(&readfds);
        FD_SET(uartPkgFd_, &readfds);
        ret = select(uartPkgFd_ + 1, &readfds, NULL, NULL, &timeout);
        if(ret <= 0) {
            STATERET(false, "startota no select return");
        }

        if ((ret = read(uartPkgFd_, buf, MAX_PACKAGE_SIZE)) <= 0) {
            STATERET(false, "startota no data read");
        }

        id = *(uint32_t*)buf;
        if (id != OTA_ACK_ID) {
            HJ_ERROR("not ota ack:%d, continue\n", id);
            continue;
        }
        else {
            uint8_t mcuid = *(buf+4);
            if(mcuid != mcuid_) {
                HJ_ERROR("mcu id not equal %x,%x\n", mcuid, mcuid_);
                STATERET(false, "mcu id not equal");
            }
            else {
                HJ_INFO("****mcu id equal, ack pass!****\n");
                STATERET(true, "ok");
            }
        }        
    }
    
    STATERET(false, "mcu ota ack timeout!");
}

void StartOta::writeUartTmrCb(const hj_bf::HJTimerEvent&) {
    uint8_t ota[] = { 0x00, 0x07, 0x00, 0x00, 
                      mcuid_, 
                      0x01, 0x00, 
                      0xff, 0xff, 0xff};

    uartHandler_ -> writePackage2Uart(ota, sizeof(ota)/sizeof(uint8_t));
}

FirmwareInfo::FirmwareInfo(int fd, UartOtaDataHandler* uartHandler):
    verCount_(3), State(fd, 5, uartHandler) {
    writeTmr_ = hj_bf::HJCreateTimer("writeFirmHead", 100 * 1000, &FirmwareInfo::writeFirmTmrCb, this, false);
}

void FirmwareInfo::init(uint32_t size, const std::string& ver) {
    size_ = size;
    ver_ = ver;
    writeFirmDone_ = false;
    parsedVerNum_.assign(verCount_, 0);
}

bool FirmwareInfo::handle() {
    HJ_INFO("****enter firminfo****\n");

    int ret = 0;
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = mcuTimeOutSec_;
    timeout.tv_usec = 0;
    uint8_t buf[MAX_PACKAGE_SIZE] = {0};
    uint32_t id = 0;

    if (!parseVer(ver_)) {
        STATERET(false, "version invalid");
    }

    for (int i = 0; i < 3; ++i) {
       HJ_INFO("vernum[%d]=%d\n", i, parsedVerNum_[i]);
    }
    HJ_INFO("bin size=%d\n", size_);
    
    while (timeout.tv_sec > 0) {
        FD_ZERO(&readfds);
        FD_SET(uartPkgFd_, &readfds);
        ret = select(uartPkgFd_ + 1, &readfds, NULL, NULL, &timeout);
        if(ret <= 0) {
            STATERET(false, "firminfo no select return");
        }

        if ((ret = read(uartPkgFd_, buf, MAX_PACKAGE_SIZE)) <= 0) {
            STATERET(false, "firminfo no data read");
        }

        id = *(uint32_t*)buf;
        if (id != dataFromMcuId_)
            continue;
        else {
            uint8_t cmd = *(buf+4);
            if (cmd != MCU_REQ_FIRMINFO_CMD) {
                if (cmd == MCU_REQ_BINDATA_CMD) {
                    HJ_INFO("write firm head done\n");
                    HJ_INFO("send bin cmd income\n");
                    break;
                } else {
                    continue;
                }
            }
            
            uint8_t mcuid = *(buf+7);
            if(mcuid != mcuid_) {
                HJ_ERROR("mcu id not equal %x,%x\n", mcuid, mcuid_);
                STATERET(false, "mcu id not equal");
            }
            
            HJ_INFO("firminfo request!\n");
            writeTmr_.start();
            continue;
        }        
    }

    //HJ_INFO("****mcu req fieminfo pass****\n");

/*
    static uint8_t firmhead[] = {0x00, 0x00, 0x00, 0x00, 
                                 SOC_SEND_FIRMINFO_CMD, 0x00, 0x00,
                                 0xff, 0xff, 0xff, 0xff,             //size
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //ver 
                                 0x0B, 0x00,
                                 0xff, 0xff, 0xff};

    UINT32_TO_BUF_LITTLE(sendDataToMcuId_, firmhead);
    uint16_t vernum[3] = {0};
    if (!parseVer(ver_, vernum)) {
        STATERET(false, "version invalid");
    }

    for (int i = 0; i < 3; ++i) {
       HJ_INFO("vernum[%d]=%d\n", i, vernum[i]);
    }

    UINT32_TO_BUF_LITTLE(size_, firmhead+7);
    UINT16_TO_BUF_LITTLE(vernum[0], firmhead+11);
    UINT16_TO_BUF_LITTLE(vernum[1], firmhead+13);
    UINT16_TO_BUF_LITTLE(vernum[2], firmhead+15);
    
    HJ_INFO("bin size=%d\n", size_);

    uartHandler_ -> writePackage2Uart(firmhead, sizeof(firmhead)/sizeof(uint8_t));
*/
    
    STATERET(true, "ok");
}

void FirmwareInfo::writeFirmTmrCb(const hj_bf::HJTimerEvent&) {
    uint8_t firmhead[] = {0x00, 0x00, 0x00, 0x00, 
                          SOC_SEND_FIRMINFO_CMD, 0x00, 0x00,
                          0xff, 0xff, 0xff, 0xff,             //size
                          0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //ver 
                          0x0B, 0x00,
                          0xff, 0xff, 0xff};

    UINT32_TO_BUF_LITTLE(sendDataToMcuId_, firmhead);
    UINT32_TO_BUF_LITTLE(size_, firmhead+7);
    UINT16_TO_BUF_LITTLE(parsedVerNum_[0], firmhead+11);
    UINT16_TO_BUF_LITTLE(parsedVerNum_[1], firmhead+13);
    UINT16_TO_BUF_LITTLE(parsedVerNum_[2], firmhead+15);

    uartHandler_ -> writePackage2Uart(firmhead, sizeof(firmhead)/sizeof(uint8_t));
}

bool FirmwareInfo::parseVer(const std::string& ver) {
    std::stringstream ss(ver);
    std::string token;

    for (int i = 0; i < verCount_; ++i) {
        if (std::getline(ss, token, '.')) {
            parsedVerNum_[i] = static_cast<uint16_t>(std::stoi(token));
        } else {
            HJ_ERROR("parse version wrong:%s", ver.c_str());
            parsedVerNum_.clear();
            return false;
        }
    }

    return true;
}

SendBin::SendBin(int fd, UartOtaDataHandler* uartHandler):
    State(fd, 120, uartHandler) {
    writeTmr_ = hj_bf::HJCreateTimer("sendBin", 100 * 1000, &SendBin::sendBinTmrCb, this, false);
}

void SendBin::init(const std::vector<uint8_t>& data) {
    if (!data_.empty())
        data_.empty();
    data_ = std::move(data);
}

void SendBin::reset() {
    State::reset();
    index_ = 0;
    retryCnt_ = 0;
}

bool SendBin::handle() {
    HJ_INFO("****enter sendbin****\n");

    int datasize = data_.size();
    //int pkgcount = datasize/MAX_DATA_CNT + 1;

    int ret = 0;
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = mcuTimeOutSec_;
    timeout.tv_usec = 0;
    uint8_t buf[MAX_PACKAGE_SIZE] = {0};
    uint32_t id = 0;
    uint32_t offsetsend = 0;
    uint32_t offsetlast = UINT32_MAX;

    while (timeout.tv_sec > 0) {
        FD_ZERO(&readfds);
        FD_SET(uartPkgFd_, &readfds);
        ret = select(uartPkgFd_ + 1, &readfds, NULL, NULL, &timeout);
        if(ret <= 0) {
            STATERET(false, "send bin no select return");
        }

        if ((ret = read(uartPkgFd_, buf, MAX_PACKAGE_SIZE)) <= 0) {
            STATERET(false, "send bin no data read");
        }

        id = *(uint32_t*)buf;
        if (id != dataFromMcuId_) {
            continue;
        }
        else {
            uint8_t cmd = *(buf+4);
            if (cmd  == MCU_REQ_BINDATA_CMD) {
                offsetsend = *(uint32_t*)(buf+7);
                HJ_INFO("offset receive:%d\n", offsetsend);
                
                if (offsetsend == offsetlast) {
                    if (++retryCnt_ <= 3) {
                        HJ_DEBUG("offset is same with last time, continue!\n");                    
                        continue;
                    }
                    retryCnt_ = 0;
                    HJ_DEBUG("offset is same with last time, but over retry, send bin again!\n");
                }
                
                if(offsetsend > offsetlast) {
                    ++index_;
                    retryCnt_ = 0;
                }

                if (offsetsend > data_.size()) {
                    HJ_ERROR("offset larger than datasize %d:%ld\n", offsetsend, data_.size());
                    STATERET(false, "offset request larger than bin size");
                }
                offsetlast = offsetsend;
                uint16_t sentlen = sendBin2uart(index_, offsetsend);
                
                if (data_.size() == offsetsend + sentlen -1) {
                    HJ_INFO("****bin send complete!****\n");
                }
            } else if (cmd == MCU_OTA_RSTCHECK_CMD) {
                uint8_t result = *(buf+7);
                if(result != 0) {
                    HJ_ERROR("mcu ota result:%d\n", result);  
                    STATERET(false, "mcu ota fail");
                } else {
                    HJ_INFO("****resultcheck success****\n");
                    STATERET(true, "ok");
                }
            } else {
                continue;
            }
        }        
    }

    STATERET(false, "send bin out of time");
}

uint16_t SendBin::sendBin2uart(uint16_t index, uint32_t offset) {
    uint8_t head[] = {0x00, 0x00, 0x00, 0x00, SOC_SEND_BINDATA_CMD, 0xff, 0xff};
    UINT32_TO_BUF_LITTLE(sendDataToMcuId_, head);
    UINT16_TO_BUF_LITTLE(index, head+5);
    
    uint8_t headlen = sizeof(head)/sizeof(uint8_t);
    uint16_t sendDataLen = 
        offset+MAX_DATA_CNT <= data_.size() ? MAX_DATA_CNT : data_.size()-offset+1;
    
    uint8_t tail[] = {0xff, 0xff, 0xff, 0xff, 0xff};
    uint8_t taillen = sizeof(tail)/sizeof(uint8_t);

    uint8_t data[headlen+sendDataLen+taillen] = {0};
    memcpy(data, head, headlen);
    memcpy(data+headlen, data_.data()+offset, sendDataLen);

    UINT16_TO_BUF_LITTLE(sendDataLen+3, tail);
    
    memcpy(data+headlen+sendDataLen, tail, taillen);
    
    if (uartHandler_->writePackage2Uart(data, sizeof(data)/sizeof(uint8_t)))
        return sendDataLen;
    else 
        return 0;
}   

UartOtaDataHandler::UartOtaDataHandler():
    uart_lastByte_(0), uart_beginFlag_(false), uart_ctrlFlag_(false),
    uart_revOffset_(0), checkSum_(0), rxPackageDataCount_(0), writefd_(-1),
    uart_(nullptr) {}

UartOtaDataHandler::~UartOtaDataHandler() {
    if(uart_)
        delete uart_;
}

void UartOtaDataHandler::reset() {
    uart_lastByte_ = 0;
    uart_beginFlag_ = false;
    uart_ctrlFlag_ = false;
    uart_revOffset_ = 0;
    checkSum_ = 0;
    rxPackageDataCount_ = 0;
}

void UartOtaDataHandler::initialize(const std::string& dev, int writefd,
                            int baud, int flowctl, int databit, 
                            int stopbit, int parity) {
    writefd_ = writefd;
    memset(m_pRxBuf, 0, MAX_PACKAGE_SIZE);
    memset(u_recvBuf_, 0, MAX_PACKAGE_SIZE);

    uart_ = new Uart(dev);
    assert(uart_);
    uart_->initialize(baud, flowctl, databit, stopbit, parity);
    uart_->run();
}


bool UartOtaDataHandler::writePackage2Uart(uint8_t *buf, uint32_t len) {
  uint32_t i;
  uint32_t sendlen;
  uint8_t *pBuf;
  uint8_t SendBuf[len*2] = {0};
  uint8_t CheckSum = 0;
  
  //HJ_INFO("send bin size:%d\n", len);

  pBuf = SendBuf;
  *pBuf++ = FRAMEHEAD;
  *pBuf++ = FRAMEHEAD;

  for (i = 0; i < len; i++) {
    if ((buf[i] == (uint8_t)FRAMECTRL) || (buf[i] == (uint8_t)FRAMEHEAD) ||
        (buf[i] == (uint8_t)FRAMETAIL)) {
      *pBuf++ = FRAMECTRL;
    }
    *pBuf++ = buf[i];
    CheckSum += buf[i];
  }

  // checksum
  if ((CheckSum == (uint8_t)FRAMECTRL) || (CheckSum == (uint8_t)FRAMEHEAD) ||
      (CheckSum == (uint8_t)FRAMETAIL)) {
    *pBuf++ = FRAMECTRL;
  }
  *pBuf++ = CheckSum;

  // Send Tail USART_FRAMETAIL USART_FRAMETAIL
  *pBuf++ = FRAMETAIL;
  *pBuf++ = FRAMETAIL;

  sendlen = pBuf - SendBuf;
  
  //HJ_INFO("uart send len:%d\n", sendlen);

  if(uart_) {
    return uart_->send(SendBuf, sendlen);
  }

  return false;
}

bool UartOtaDataHandler::analyzePackage() {
  int byte_num = uart_->recv(u_recvBuf_);
  if (byte_num <= 0) {
    HJ_ERROR("no read from uart\n");
    return false;
  }

  uint8_t data;
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
        uint32_t id = *(uint32_t *)m_pRxBuf;
        //HJ_DEBUG(">>>>recv from uart, id=%d\n", id);
        if(id == OTA_ACK_ID || 
           id == OTA_MCU_DATA_ID ||
           id == OTA_LAMP_DATA_ID) {
            write(writefd_, m_pRxBuf, rxPackageDataCount_+1);
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

McuOtaRun::McuOtaRun(std::string uart, int baud, int flowctl, int databit, int stopbit, int parity):
    uartdev_(uart), curState_(nullptr), uartHandler_(nullptr), 
    isOtaRun_(false), isOtaSucc_(false), mcuId_(0) {
    
    if(pipe(rwFd_) != 0) {
        HJ_ERROR("pipe init fail\n");
        return;
    }
    uartInit(uartdev_, rwFd_[1], baud, flowctl, databit, stopbit, parity);
    stateMachineConstruct();
}

McuOtaRun::~McuOtaRun() {
    isOtaRun_ = false;
    joinThreads();

    if(!states_.empty()) {
        for (auto it = states_.begin(); it != states_.end(); ++it) {
            delete it->second;
        }
    }

    ::close(rwFd_[0]);
    ::close(rwFd_[1]);

    if(uartHandler_)
        delete uartHandler_;
}

bool McuOtaRun::init(uint8_t type, const std::string& vernum, const std::string& binfile) {
    
    /*check version format x.x.x*/
    std::regex pattern(R"(\b\d+\.\d+\.\d+\b)");
    if (!std::regex_match(vernum, pattern)) {
        HJ_ERROR("version format wrong: %s\n", vernum.c_str());
        return false;
    }

    otaType_ = static_cast<ota_type>(type);

    if (otaType_ == MCU) {
        mcuId_ = MCU_MAIN_BOARD;
    } else if (otaType_ = LAMPBOARD) {
        mcuId_ = MCU_LAMP_BOARD;
    } else {
        HJ_ERROR("unsurpport type:%d\n", type);
        return false;
    }
    
    verNum_ = vernum;
    binFile_ = binfile;
    binData_.empty();


    if (!loadBinFile()) {
        HJ_ERROR("load bin fail\n");
        return false;
    }

    stateMachineInit(mcuId_);
    return true;
}

bool McuOtaRun::loadBinFile() {
    std::ifstream file(binFile_, std::ios::binary);
    if (!file) {
        HJ_ERROR("Failed to open file\n");
        return false;
    }

    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    binData_.assign(fileSize, 0);
    if (!file.read(reinterpret_cast<char*>(binData_.data()), fileSize)) {
        HJ_ERROR("Failed to read file\n");
        return false;
    }

    HJ_INFO("bin size:%ld\n", fileSize);
    HJ_INFO("data size:%d\n", binData_.size());

    return true;
}

void McuOtaRun::uartInit(std::string uart, int fd, int baud,
              int flowctl, int databit, int stopbit, int parity) {
    uartHandler_ = new UartOtaDataHandler;
    assert(uartHandler_);
    uartHandler_->initialize(uart, fd, baud, flowctl, databit, stopbit, parity);
}

void McuOtaRun::stateMachineConstruct() {
    states_.insert(std::make_pair("startota", 
                   new StartOta(rwFd_[0], uartHandler_)));
    states_.insert(std::make_pair("firminfo", 
                   new FirmwareInfo(rwFd_[0], uartHandler_)));
    states_.insert(std::make_pair("sendbin",
                   new SendBin(rwFd_[0], uartHandler_)));
   
    states_["startota"]->setNextSuccState(states_["firminfo"]);

    states_["firminfo"]->setNextSuccState(states_["sendbin"]);

    curState_ = states_["startota"];
}

void McuOtaRun::stateMachineInit(uint8_t id) {
    for (auto it = states_.begin(); it != states_.end(); ++it) {
        it->second->setMcuOtaId(id);
    }
    
    FirmwareInfo* firm = dynamic_cast<FirmwareInfo*>(states_["firminfo"]);
    firm->init(binData_.size(), verNum_);

    SendBin* sendbin = dynamic_cast<SendBin*>(states_["sendbin"]);
    sendbin->init(binData_);
}

void McuOtaRun::runMachineState() {
    while(curState_ && isOtaRun_) {
        if(curState_->handle()) {
            curState_ = curState_->nextSuccState();
        } else {
            HJ_ERROR("%s\n", curState_->getRstMsg().c_str());
            /*
              Next fail state is always null.
              Ota ends with failure if any state returns false and report it.
            */
            if (pubMcuOtaRstCb_)
                pubMcuOtaRstCb_(otaType_, false, curState_->getRstMsg());
            
            curState_ = curState_->nextFailState();
        }
    }

    isOtaSucc_ = states_["sendbin"]->getResult();

    for (auto it = states_.begin(); it != states_.end(); ++it) {
        it->second->reset();
    }

    isOtaRun_ = false;
    curState_ = states_["startota"];

    HJ_INFO("machine state run end, ota result:%d\n", isOtaSucc_);

    if (isOtaSucc_ && pubMcuOtaRstCb_)
        pubMcuOtaRstCb_(otaType_, true, "ok");
}

void McuOtaRun::runUartHandler() {
    int ret = true;
    while(isOtaRun_) {
        uartHandler_->analyzePackage();
    }

    uartHandler_->reset();
    HJ_INFO("uart handler run end\n");
}

void McuOtaRun::otaRun() {
    HJ_INFO("mcu ota run!\n");

    isOtaRun_ = true;
    
    runUartHandler_ = std::thread(&McuOtaRun::runUartHandler, this);
    runMachineState_ = std::thread(&McuOtaRun::runMachineState, this);
}

void McuOtaRun::joinThreads() {
    HJ_INFO("stop mcu ota...\n");

    if (runMachineState_.joinable()) {
        HJ_INFO("mcu ota machine state exit...\n");
        runMachineState_.join();
        HJ_INFO("mcu ota machine state exit success!\n");
    }

    if (runUartHandler_.joinable()) {
        HJ_INFO("mcu ota uart handler exit...\n");
        runUartHandler_.join();
        HJ_INFO("mcu ota uart handler exit success!\n");
    }
    
    HJ_INFO("mcu ota stop!\n");
}

bool McuOtaRun::resetMcu() {
    HJ_INFO("****ota reset!****\n"); 
    
    if (logger)
        hj_cst_log_del(logger);

    restTmr_ = hj_bf::HJCreateTimer("resetMcu", 100 * 1000, &McuOtaRun::resetTimerCb, this);

    return true;
}

void McuOtaRun::resetTimerCb(const hj_bf::HJTimerEvent&)
{
    uint8_t otaRst[] = {0x02, 0x07, 0x00, 0x00, 
                        SOC_QUIT_MCUOTA_CMD, 0x00, 0x00,
                        0x03, 0x00,
                        0xff, 0xff, 0xff};
                        
    uartHandler_->writePackage2Uart(otaRst, sizeof(otaRst)/sizeof(uint8_t));
}

McuOta::McuOta(const rapidjson::Value& json_conf): 
    hj_bf::Function(json_conf), mcuOtaRun_(nullptr) {   
    
    if (json_conf.HasMember("log") && json_conf["log"].IsString()) {
        std::string file = json_conf["log"].GetString();
        logger = hj_cst_log_add(file.data(), INFO_LOG, 20*1024*1024, 2);
    }

    if (json_conf.HasMember("mcu_para") && json_conf["mcu_para"].IsObject()) {
        auto mcupara = json_conf["mcu_para"].GetObject();
        std::string port = mcupara["port"].GetString();
        int baud = mcupara["baudRate"].GetInt();
        int flow_ctrl = mcupara["flowCtl"].GetInt();
        int databits = mcupara["dataBits"].GetInt();
        int stopbits = mcupara["stopBits"].GetInt();
        std::string parity = mcupara["parity"].GetString();
    
        mcuOtaRun_ = new McuOtaRun(port, baud, flow_ctrl, databits, stopbits, parity[0]);
    } else {
        mcuOtaRun_ = new McuOtaRun("/dev/ttyS4", 460800, 0, 8, 1, 'N');
    }

    assert(mcuOtaRun_);
    
    mcuOtaRun_->setMcuOtaResultPubFunc(boost::bind(&McuOta::pubMcuOtaResult, this,
                                         boost::placeholders::_1,
                                         boost::placeholders::_2,
                                         boost::placeholders::_3));

    mcuctl_pub_ = hj_bf::HJAdvertise<std_msgs::Bool>("mcuInterCtl", 1);
    mcuOtaRstPub_ = hj_bf::HJAdvertise<hj_interface::OtaUpgradeStatus>("/system/eventStatusRep", 1);
    mcuOtaCtl_ = hj_bf::HJSubscribe("/system/eventNotify", 1, &McuOta::mcuOtaCtlCallback, this);
    mcuOta_ = hj_bf::HJSubscribe("mcuOtaReady", 1, &McuOta::mcuOtaReadyCallBack, this);

    mcuOtaReadyTmr_ = hj_bf::HJCreateTimer("mcuOtaReadyTimer", 5 * 1000 * 1000, &McuOta::mcuOtaReadyTimerCallBack, this);
    mcuOtaReadyTmr_.stop();
}

McuOta::~McuOta() {
    if(mcuOtaRun_)
        delete mcuOtaRun_;
    hj_cst_log_del(logger);
}

void McuOta::mcuOtaCtlCallback(const hj_interface::OtaUpgradeData::ConstPtr& msg) {
    
    HJ_INFO("receive ota msg:\n%s, %d\n", msg->todo.c_str(), msg->module);

    if (msg->todo == "Reboot") {
        if (msg->module == MCU) 
            mcuOtaRun_->resetMcu();
        
        return;
    }

    if (msg->todo == "UpdateOTA") {
        if (msg->module != MCU && msg->module != LAMPBOARD) {
            HJ_INFO("ignore module:%d\n", msg->module);
            return;
        }
        if (msg->stage != 1) {
            HJ_INFO("ignore stage:%d\n", msg->stage);
            return;
        }
    }

    if (msg->data.addr.empty() || msg->data.ver.empty()) {
        HJ_ERROR("mcu ota para invalid\n");
        pubMcuOtaResult(static_cast<ota_type>(msg->module), false, "mcu ota para invalid");
        return;
    }

    std::string ver = msg->data.ver;
    std::string binfile = msg->data.addr;

    if(mcuOtaRun_->isOtaRun()) {
        HJ_INFO("mcu ota is running\n");
        pubMcuOtaResult(static_cast<ota_type>(msg->module), false, "another mcu ota is running");
        return;
    }

    HJ_INFO("trigger mcu ota, module:%d, ver:%s, bin:%s\n",msg->module, ver.c_str(), binfile.c_str());
    
    if (!mcuOtaRun_->init(msg->module, ver, binfile)) {
        pubMcuOtaResult(static_cast<ota_type>(msg->module), false, "mcu ota init fail");
        return;
    }

    /*notify mcu to stop uart read and trigger timer*/
    std_msgs::Bool notify;
    notify.data = false;
    mcuctl_pub_.publish(notify);
    mcuOtaReadyTmr_.start();
    return;
}

void McuOta::mcuOtaReadyTimerCallBack(const hj_bf::HJTimerEvent&) {
    if (mcuOtaRun_->isOtaRun()) {
        HJ_ERROR("mcu ota ready time out, but is running\n");
    } else {
        HJ_ERROR("mcu ota ready time out\n");
        pubMcuOtaResult(mcuOtaRun_->getOtaType(), false, "mcu ota ready time out");
    }
    mcuOtaReadyTmr_.stop();
}

void McuOta::mcuOtaReadyCallBack(const std_msgs::Bool::ConstPtr& msg) {
    HJ_INFO("mcu can ota:%d\n", msg->data);
    mcuOtaReadyTmr_.stop();
    if(!msg->data) {
        pubMcuOtaResult(mcuOtaRun_->getOtaType(), false, "mcu cannot ota");
        return;
    }

    if(tmtx_.try_lock_for(std::chrono::milliseconds(2000))) {
        mcuOtaRun_->otaRun();
        tmtx_.unlock();
        otaThreadJoin_ = std::thread(&McuOta::otaThreadJoinFunc, this);
        otaThreadJoin_.detach();
    } else {
        HJ_ERROR("threads join time out, quit ota!\n");
        pubMcuOtaResult(mcuOtaRun_->getOtaType(), false, "threads join time out, quit ota");
    }
}

void McuOta::otaThreadJoinFunc() {
    {
        std::unique_lock<std::timed_mutex> lc(tmtx_);
        mcuOtaRun_->joinThreads();
    }

    HJ_INFO("mcu ota threads join!\n");
    
    /*notify mcu to continue uart read*/
    /*
    std_msgs::Bool notify;
    notify.data = true;
    mcuctl_pub_.publish(notify);
    */
}

void McuOta::pubMcuOtaResult(ota_type type, bool result, const std::string& msg) {
    hj_interface::OtaUpgradeStatus pubmsg;

    pubmsg.todo = "UpdateOTA";
    pubmsg.stage = 1;
    pubmsg.module = type;
    pubmsg.ret = result ? 0 : -1;
    pubmsg.msg = msg;
    mcuOtaRstPub_.publish(pubmsg);

    HJ_INFO("pub module:%d result:%d, %s\n", pubmsg.module, pubmsg.ret, pubmsg.msg.c_str());
}

}
//namespace collect_node_mcu_ota