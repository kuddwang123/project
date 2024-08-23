#ifndef PUBLIC_MACRO_H
#define PUBLIC_MACRO_H

#define BACKUP_PATH              "/data/hj/config/"
#define LOGIC_DIR                "/userdata/logic/"
#define CLOUD_VERSION_FILE_PATH       LOGIC_DIR    "cloud_version.json"
#define PARA_CONFIG_FILE_PATH         LOGIC_DIR    "paraConfig.json"
#define BACKUP_PARA_CONFIG_FILE_PATH  BACKUP_PATH  "paraConfig.json" 
#define CLEAN_RECORD_DIR              LOGIC_DIR    "clean_record"
// #define CLEAN_RECORD_DIR              "/tmp/logic/clean_record"
#define TIME_ZONE_FILE_PATH           "/userdata/.root/etc/tzm.json"

/******************保存清洁记录的最大天数*********************/
const int32_t CLEAN_RECORD_MAX_DAYS = 28;
#define MAP_DIR                    "/userdata/hj/maps/"
#define MAP_FILE_PATH               MAP_DIR      "point_cloud.bin"


/*****************出入水检测宏*********************/
#define USE_CHECK_WATER_SENSOR

/******************coredump测试宏*********************/
// #define DEBUG_COREDUMP

/******************演示宏*********************/
// #define FOR_2024_7_2_DEMO

#endif // PUBLIC_MACRO_H