#!/bin/sh

LOG_DIR=/userdata/hj/log/monitor_log

if [ ! -d ${LOG_DIR} ];then
  mkdir -p ${LOG_DIR}
else
  echo ${LOG_DIR}
fi

COLLECT_NODE=collect_node
SLAM_NODE=slam_node
UTILS_NODE=utils_node
PLANNING_NODE=planning_node
LOG_RECORD_NODE=log_recorder
MIDDLEWARE_NODE=middleware_node

PID_COLLECT_NODE=`pidof ${COLLECT_NODE}`
PID_SLAM_NODE=`pidof ${SLAM_NODE}`
PID_UTILS_NODE=`pidof ${UTILS_NODE}`
PID_PLANNING_NODE=`pidof ${PLANNING_NODE}`
PID_LOG_RECORD_NODE=`pidof ${LOG_RECORD_NODE}`
PID_MIDDLEWARE_NODE=`pidof ${MIDDLEWARE_NODE}`

PID_STR=''

if [ ! $PID_COLLECT_NODE ]; then
  echo "COLLECT_NODE process IS NULL"
else
  echo "pid of ${COLLECT_NODE} is ${PID_COLLECT_NODE}"
  PID_STR=`echo -e "-p $PID_COLLECT_NODE"`
fi

if [ ! $PID_SLAM_NODE ]; then
  echo "SLAM_NODE process IS NULL"
else
  echo "pid of ${SLAM_NODE} is ${PID_SLAM_NODE}"
  PID_STR=`echo -e "-p $PID_SLAM_NODE" $PID_STR`
fi

if [ ! $PID_UTILS_NODE ]; then
  echo "UTILS_NODE process IS NULL"
else
  echo "pid of ${UTILS_NODE} is ${PID_UTILS_NODE}"
  PID_STR=`echo -e "-p $PID_UTILS_NODE" $PID_STR`
fi

if [ ! $PID_PLANNING_NODE ]; then
  echo "PLANNING_NODE process IS NULL"
else
  echo "pid of ${PLANNING_NODE} is ${PID_PLANNING_NODE}"
  PID_STR=`echo -e "-p $PID_PLANNING_NODE" $PID_STR`
fi

if [ ! $PID_LOG_RECORD_NODE ]; then
  echo "LOG_RECORD_NODE process IS NULL"
else
  echo "pid of ${LOG_RECORD_NODE} is ${PID_LOG_RECORD_NODE}"
  PID_STR=`echo -e "-p $PID_LOG_RECORD_NODE" $PID_STR`
fi

if [ ! $PID_MIDDLEWARE_NODE ]; then
  echo "MIDDLEWARE_NODE process IS NULL"
else
  echo "pid of ${MIDDLEWARE_NODE} is ${PID_MIDDLEWARE_NODE}"
  PID_STR=`echo -e "-p $PID_MIDDLEWARE_NODE" $PID_STR`
fi

if [ ! "$PID_STR" ]; then
  echo "all process IS NULL"
else
  while true; do
    `pidstat -urds -h $PID_STR 1 1 >> ${LOG_DIR}/top.log`
    ROW_NUM=`cat $LOG_DIR/top.log  | wc -l`

    if [ "$ROW_NUM" -ge "10000" ];then
      cp ${LOG_DIR}/top.log ${LOG_DIR}/top_bak.log
      rm -rf $LOG_DIR/top.log
    fi
    # sleep 1
  done
fi


echo "----- get top info done: ${LOG_DIR} -------"