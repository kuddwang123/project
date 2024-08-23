#!/usr/bin/env bash
# 守护进程，每隔5秒守护一次
ABSOLUTE_PATH="/home/robot/data/hj"

source ${ABSOLUTE_PATH}/devel/setup.bash
export LD_LIBRARY_PATH=/home/robot/opencv4.2.0/lib:/${ABSOLUTE_PATH}/lib:${LD_LIBRARY_PATH}

# 状态码定义
FAILED=1
NORMAL=4
COLLECT_NODE_ERROR=1998
UTILS_NODE_ERROR=2998
SLAM_NODE_ERROR=3998
PLANNING_NODE_ERROR=4998
MIDDLEWARE_NODE_ERROR=5998
LOG_RECORD_NODE_ERROR=6998

# 节点名称
COLLECT_NODE=collect_node
SLAM_NODE=slam_node
UTILS_NODE=utils_node
PLANNING_NODE=planning_node
LOG_RECORD_NODE=log_recorder
MIDDLEWARE_NODE=middleware_node

COLLECT_NODE_RESTART_COUNT=0
SLAM_NODE_RESTART_COUNT=0
UTILS_NODE_RESTART_COUNT=0
PLANNING_NODE_RESTART_COUNT=0
LOG_RECORD_NODE_RESTART_COUNT=0
MIDDLEWARE_NODE_RESTART_COUNT=0

function ros_pub() {
  cur_timestamp=$((`date '+%s'`*1000+10#`date '+%N'`/1000000)) # 10#表明十进制数据
  rosservice call /hj_health_monitor $cur_timestamp $1 $2   # timestamp, code_val, status
}

# $1:节点名称 $2:重启次数 $3:错误码
function restart_func()
{
  # 检查文件是否存在
  if [ -f "/tmp/logging/log_err1" ]; then
    log_err0_CurrTime=`ls -l --time-style  '+%Y-%m-%d %H:%M:%S' /tmp/logging/log_err0 | awk '{print $6,$7}'`
    log_err0_CurrTimestamp=`date -d "$log_err0_CurrTime" +%s`
    log_err1_CurrTime=`ls -l --time-style  '+%Y-%m-%d %H:%M:%S' /tmp/logging/log_err1 | awk '{print $6,$7}'`
    log_err1_CurrTimestamp=`date -d "$log_err1_CurrTime" +%s`

    if [ $log_err0_CurrTimestamp -gt $log_err1_CurrTimestamp ]; then
      log_err_file="/tmp/logging/log_err0"
    else
      log_err_file="/tmp/logging/log_err1"
    fi
  else
    log_err_file="/tmp/logging/log_err0"
  fi 
  
  count=$2
  local res=`ps -ef | grep $1 | grep -v grep | wc -l`
  if [ $res -eq 0 ]; then
    count=$(($2+1))
    if [ $count -eq 1 ]; then
      ros_pub $3 $FAILED
    fi
    
    if [ $2 -gt 3 ]; then
      echo "$1 Too many restarts, restart whole system..." >> $log_err_file
      ${ABSOLUTE_PATH}/bin/hj_manager.sh "yes"
      exit 1
    else 
      echo "$1 is not running, restarting... try count:$2" >> $log_err_file
      `${ABSOLUTE_PATH}/bin/$1 > /dev/null &`
    fi
  else
    if [ $2 -gt 0 ]; then
      ros_pub $3 $NORMAL
    fi
  fi
  return $count
}

while true
do
  restart_func $COLLECT_NODE $COLLECT_NODE_RESTART_COUNT $COLLECT_NODE_ERROR
  COLLECT_NODE_RESTART_COUNT=$?
  restart_func $SLAM_NODE $SLAM_NODE_RESTART_COUNT $SLAM_NODE_ERROR
  SLAM_NODE_RESTART_COUNT=$?
  restart_func $UTILS_NODE $UTILS_NODE_RESTART_COUNT $UTILS_NODE_ERROR
  UTILS_NODE_RESTART_COUNT=$?
  restart_func $PLANNING_NODE $PLANNING_NODE_RESTART_COUNT $PLANNING_NODE_ERROR
  PLANNING_NODE_RESTART_COUNT=$?
  restart_func $LOG_RECORD_NODE $LOG_RECORD_NODE_RESTART_COUNT $LOG_RECORD_NODE_ERROR
  LOG_RECORD_NODE_RESTART_COUNT=$?
  restart_func $MIDDLEWARE_NODE $MIDDLEWARE_NODE_RESTART_COUNT $MIDDLEWARE_NODE_ERROR
  MIDDLEWARE_NODE_RESTART_COUNT=$?

  sleep 5    #守护进程运行间隔
done
