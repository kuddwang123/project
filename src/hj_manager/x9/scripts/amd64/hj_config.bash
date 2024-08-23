
mkdir /tmp/log
export ROS_LOG_DIR=/tmp/log
export HJ_LOG_CONFIG_PATH=/home/wangqing/work_space/hj_workspace/src/hj_manager/x9/config/amd64/hj_log.config
export LOG_RECORDER_NEW_PATH=/home/wangqing/work_space/hj_workspace/src/hj_manager/x9/config/amd64/log_recorder.json
export BIG_DATA_CONFIG_FILE=/home/wangqing/work_space/hj_workspace/src/hj_manager/x9/config/amd64/big_data_config.json
export LD_LIBRARY_PATH=/home/wangqing/work_space/hj_workspace/src/hj_interface/platforms/amd64:${LD_LIBRARY_PATH}:/home/wangqing/work_space/hj_workspace/src/thirdparty/platforms/amd64/awsiot/
rosparam set /hj_so_path "${node_path}/devel/lib"
rosparam set /hj_config_path "${node_path}/src"
echo $LD_LIBRARY_PATH

