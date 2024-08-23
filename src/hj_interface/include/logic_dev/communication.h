#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/**************************health_monitor******************************/
#define  BOOTING_RESULT_2_BOOTING               "/booting_result_internal"
#define  BOOTING_RESULT_NOTIFY                  "/booting_result_sub"
#define  UPLOAD_IOT                             "/upload_iot"
#define  STATE_DEAL                             "/state_deal"
#define  APP_ONLINE                             "/AppOnline"

/**************************task_manager********************************/
#define  SIGNAL_TASK                            "/signal_task_sub"
#define  PERIODIC_TASK                          "/periodic_task_sub" 
#define  CLEAN_TASK                             "/navi_clean_work"
#define  CLEAN_ACTION                           "/navi_clean_action"
#define  WORK_RESULT                            "/navi_work_result_return"
#define  WORKING_STATE                          "/navi_working_state"
#define  SNORKELING                             "/air_bag"
#define  SNORKELING_RESULT                      "/snorkeling_result"
#define  TOPIC_OUT_WATER_DETECT                 "/outwater_detect"
#define  SLAM_ACTION_SERVICE                    "/slam_action_service"
#define  NAVI_ACTION_SERVICE                    "/navi_action_service"
#define  SLAM_NAVI_ACTION_RESULT_SERVICE        "/slam_navi_action_result_service"
#define  ROBOT_BACK_WORK_SERVICE                "/robot_back_work_service"
#define  UPLOAD_FILE_TOPIC                      "/upload/file"
#define  TOPIC_PUMP_CONTROL                     "/control/pumpTurnOn"
/**************************parse_command*******************************/
#define  PARSE_COMMAND                          "/parse_command_sub"
#define  LORA_REMOTE_CONTROL                    "/lora_remotectl"
#define  DEVICE_BUTTON_CMD                      "/device_button_cmd"    
#define  MCU_CMD_RESP                           "/mcu_cmd_resp"
#define  NAV_MOTOR                              "/nav_motor"
#define  TO_MIDDLE                              "/to_middle"
#define  TO_COLLECT                             "/to_collect"
#define  REQ_FROM_APP                           "/ReqFromApp"
#define  RESP_TO_APP                            "/RespToApp"
#define  REPORT_APP                             "/ReportApp"
#define  TO_PARSE_NODE                          "/to_parse_node"
#define  TO_OTHER_NODE                          "/to_other_node"
#define  IOT_DELETE_SHADOW_SERVICE              "/IotDeleteShadow"
#define  TOPIC_SET_LED_AFFECT                   "/set_led_affect"
#define  TOPIC_DEV_INFO                         "/devInfoReport"
constexpr const char* kSysActionReq = "/sys_action_req";
constexpr const char* kSysActionResp = "/sys_action_resp";
/******************************OTA************************************/
#define TOPIC_OTA_SEND_BIN_INFO                 "/system/sendBinInfo"
#define TOPIC_OTA_RECV_BIN_LOCATION             "/system/recvBinLocation"  
#define TOPIC_OTA_EVENT_NOTIFY                  "/system/eventNotify"
#define TOPIC_OTA_EVENT_STATUS_REP              "/system/eventStatusRep"
#define TOPIC_OTA_DOWNLOAD_PACK_RESULT          "/system/dlResult"
#define TOPIC_OTA_DOWNLOAD_PACK_PROGRESS        "/system/dlProgressing"
#define TOPIC_OTA_MCU_VERSION_DATA              "/mcuVer_chatter"
#define TOPIC_FLUSH_DEVINFO                     "/system/updateDevInfo"
/******************************Sensor************************************/
#define BAT_CHATTER                                   "bat_chatter"
#define TOPIC_DEPTH_CHATTER                           "depth_chatter"
#define TOPIC_MOTOR_CUR                               "/motor_cur"
#define TOPIC_TURBIDITY_DATA                          "turbidity_data"
#define TOPIC_MOTOR_CHATTER                           "motor_chatter"
#define TOPIC_PUMP_MOTOR_SPEED_CHATTER                "/pumpMotorSpeed_chatter"
#define TOPIC_IMU_CHATTER "imu_chatter"
#define TOPIC_MACHINE_ON_TIMES "/machine_on/times"
#define TOPIC_CLEAN_MODE  "/clean_mode"
#define TOPIC_MIDDLEWARE_TASK                       "/middleware_task"
/******************************Alarm************************************/
#define TOPIC_HEALTH_MONITOR "/hj_health_monitor"
#endif // COMMUNICATION_H