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
