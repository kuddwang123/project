#pragma once

#include <boost/function.hpp>
#include <string>

extern void* ota_logger;

namespace aiper_ota {

typedef boost::function<void(int state, int progress, std::string msg, int error)> 
    otaStatusReportFunc;

typedef boost::function<void(int state, int angostate, int ledstate, int basestate, int socstate, std::string msg, int error)>
    otaResultReportFunc;

typedef boost::function<void(int)>
    dlProgressFunc;

typedef boost::function<void(std::string)>
    buryPointUploadFunc;

enum EndState {
    DOWNLOAD_STATE,
    UNPACK_STATE,
    UPGRADE_SUCC_STATE,
    UPGRADE_FAIL_STATE
};

enum OtaModule {
    none = 0,
    charger,
    ledboard,
    baseboard,
    soc,
    ango
};

enum {
    BP_OK = 0,
    BP_UNPACK_FAIL,
    BP_MCU_LED_OTA_FAIL,
    BP_MCU_BASE_OTA_FAIL,
    BP_SOC_OTA_FAIL,
    BP_INTRRUPT_BY_MANUAL,
    BP_ALREADY_IN_OTA,
    BP_FAIL_TO_DL_NO_SPACE,
    BP_ENTER_OTA_REJECT_BY_MID,
    BP_DLFAIL_MD5_MISMATCH,
    BP_DLFAIL_NETWORK_FAIL
};

typedef struct {
    int mode_;
    int timeout_;
    std::string url_;
    std::string md5_;
    std::string nextver_; 
}DlPara;

typedef struct {
    EndState endState_;
    OtaModule failModule_;
}EndPara;

typedef struct {
    uint64_t controlLogId_;
    std::string curVer_;
    std::string nextVer_;
    std::string errorMsg_;
}CloudVersion;

} //namespace aiper_ota
