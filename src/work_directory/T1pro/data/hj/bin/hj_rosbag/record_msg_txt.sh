#!/bin/bash
source /home/robot/data/hj/devel/setup.bash

# Usage:./record_msg_txt.sh
# This script will record the messages of the topics ["/turbidity_data", "/ulsound_chatter", "/range_data",...] in separate text files.
# usage:


limit_log_size()
{
    local logfile=$1
    local destfile=$2
    local maxsize=$3

    if [ ! -f "$logfile" ]; then
        echo "" > $logfile
    fi
    
    filesize=`du $logfile | awk '{print $1}'`
    
    if [ $filesize -ge $maxsize ]; then
        echo "$filesize k >= $maxsize k"
        
        if [ -f $destfile ]; then
            rm $destfile -rf
        fi
        
        cp -rf $logfile $destfile
        
        # 清空之前内容
        echo "" > $logfile
    fi
}

topics=`cat topic_config`
for topic in $topics
do
    echo "Processing topic: $topic"
    rostopic echo  -p ${topic} >> ${topic:1}.txt 2>&1 &
done

#for topic in $topics
# for ((i=0; i<${#topics[@]}; i++));
# do
#     echo "Processing topic: ${topics[$i]}"
#     rostopic echo  -p ${topics[$i]} > ${topics[$i]:1}.txt &
# done

while true
do
  DiskSize=`df -h |grep "userdata" | awk '{print $4}'`
  echo "Disk usage: $DiskSize"
  if [[ $DiskSize < "50M" ]]; then
    echo "Disk usage is less than 50MB, recording is stopped."
    PID=`ps -ef | grep -E "rostopic" | grep -v grep | awk '{print $2}'`
    for item  in $PID
    do
        kill -2 $item
        echo "kill -2 $item"
    done 
    break
  fi
  
  for topic in $topics
  do
      echo "check topic: $topic"
      limit_log_size "${topic:1}.txt" "${topic:1}_bk.txt" 200  # 限制日志大小为200K
  done
  

  sleep 5    #运行间隔
done
