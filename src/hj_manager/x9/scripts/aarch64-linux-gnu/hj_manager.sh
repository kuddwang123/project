#!/usr/bin/env bash
ABSOLUTE_PATH="/data/hj"

echo $#
if [ $# == 1 ]; then
  ABSOLUTE_PATH=$1
  echo ${ABSOLUTE_PATH}
fi

source /opt/ros/melodic/setup.bash
# export LD_LIBRARY_PATH=/${ABSOLUTE_PATH}/lib:${LD_LIBRARY_PATH}:/${ABSOLUTE_PATH}/lib/thirdparty
export LD_LIBRARY_PATH=/${ABSOLUTE_PATH}/lib:/${ABSOLUTE_PATH}/lib/slam_node/slam_node:/${ABSOLUTE_PATH}/lib/ziplib:${LD_LIBRARY_PATH}

export HJ_LOG_CONFIG_PATH="${ABSOLUTE_PATH}/config/hj_log.config"
export LOG_RECORDER_NEW_PATH="${ABSOLUTE_PATH}/config/log_recorder.json"
export BIG_DATA_CONFIG_FILE="${ABSOLUTE_PATH}/config/big_data_config.json"
#export HJ_ALL_LOG_CLOSE=close
#export HJ_LOG_CLOSE_collect_node=close
mkdir -p /tmp/logging
mkdir -p /userdata/hj/log/logging
rm /tmp/logging/log_err

if [ ! -d "/userdata/hj/maps" ]; then
    mkdir -p /userdata/hj/maps
fi

# 设置基础路径文件夹
base_folder="/tmp/planning_log"

# 判断基础路径文件夹是否存在，如果不存在则创建
if [ ! -d "$base_folder" ]; then
	    mkdir -p "$base_folder"
fi

cd "$base_folder"
rm -rf *

# filename="${ABSOLUTE_PATH}/point_cloud.bin"

# # 检查文件是否存在
# if [ -e "$filename" ]; then
#     # 删除文件
#     rm "$filename"
#     echo "File '$filename' exists and has been removed."
# else
#     echo "File '$filename' does not exist."
# fi


# 使用当前时间创建子文件夹
sub_folder="$base_folder/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$sub_folder"

# 将新创建的子文件夹路径设置为环境变量 HJ_uwr_LOG_PATH
export HJ_uwr_LOG_PATH="$sub_folder"

# 输出日志文件夹路径
echo "HJ_uwr_LOG_PATH is set to: $HJ_uwr_LOG_PATH"
rm -rf /dev/shm/MINOS_SHM
while true; do
    COUNT=$(ps -A | grep roscore | wc -l)
    if [ "$COUNT" != "0" ]; then
        break
    fi
    roscore &
    sleep 1 # wait roscore running
done


PID=`ps -ef | grep -E "monitor.sh|middleware_node|planning_node|slam_node|utils_node|collect_node|log_recorder|rosctl_ser" | grep -v grep | awk '{print $1}'`
for item  in $PID
do
	kill -s 9 $item
	echo "kill -s 9 $item"
done

PID=`ps -ef | grep -E "collect_node" | grep -v grep | awk '{print $1}'`
for item  in $PID
do
	kill -s 2 $item
	echo "kill -s 2 $item"
	IS_EXIST=`ps -ef | grep -E "collect_node" | grep -v grep`
	while [ -n "$IS_EXIST"  ]
	do
			echo "steal exit"
			sleep 1
            IS_EXIST=`ps -ef | grep -E "collect_node" | grep -v grep`
	done
done


retry_cnt=0;
while [ __1 == __1 ]
do
    if [ $retry_cnt -gt 10 ]; then
        echo "rosparam set fail, exit"
        exit -1
    fi
    param_hj_so=$(rosparam set /hj_so_path "${ABSOLUTE_PATH}/lib" 2>&1)
    param_hj_config=$(rosparam set /hj_config_path "${ABSOLUTE_PATH}/config" 2>&1)

    if [[ "$param_hj_so" == *"ERROR"* || "$param_hj_config" == *"ERROR"* ]]; then
        echo "rosparam set error, retry"
        ((retry_cnt++))
        sleep 1
    else
        echo "rosparam set success"
        break
    fi
done

retry_cnt=0;
while [ __1 == __1 ]
do
    if [ $retry_cnt -gt 10 ]; then
        echo "rosparam get fail, exit"
        exit -1
    fi
    param_hj_so=$(rosparam get /hj_so_path 2>&1)
    param_hj_config=$(rosparam get /hj_config_path 2>&1)

    if [[ "$param_hj_so" == *"ERROR"* || "$param_hj_config" == *"ERROR"* ]]; then
        echo "rosparam get error, retry"
        ((retry_cnt++))
        sleep 1
    else
        echo "rosparam get success"
        break
    fi
done


#rosparam set /hj_so_path "${ABSOLUTE_PATH}/lib"
#rosparam set /hj_config_path "${ABSOLUTE_PATH}/config"
#./log_recorder > /dev/null &

${ABSOLUTE_PATH}/bin/collect_node > /dev/null &
sleep 2  # wait collect_node running, 顺序加载依赖库，防止i/o抢占导致初始化更慢
${ABSOLUTE_PATH}/bin/middleware_node > /dev/null &
sleep 2
${ABSOLUTE_PATH}/bin/planning_node > /dev/null &
${ABSOLUTE_PATH}/bin/slam_node > /dev/null &
${ABSOLUTE_PATH}/bin/utils_node > /dev/null &
rosctl_ser 13001 &
#sleep 2
#${ABSOLUTE_PATH}/bin/monitor.sh  &
