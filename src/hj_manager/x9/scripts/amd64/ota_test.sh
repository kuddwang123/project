#!/usr/bin/env bash


#sleep 30
TEMP_FILE=/tmp/ota_topic
# RESULT_FILE=/userdata/hj/log/ota_result
RESULT_FILE=/tmp/ota_result
content=""
count=0

result=$(cat /home/wangqing/middleware_node_ros1.log | grep -a "OtaResultReport" )
echo $result > /tmp/check_result

result2=$(grep "OtaResultReport" /tmp/check_result | tail -n 1 | grep -o '"state":[0-9]*' | cut -d':' -f2)
echo $result2 
echo $result2 >/tmp/check_result2
echo "File is ff."
#tail -n 3 /tmp/check_result2 | head -n 1
ota_result=$(awk 'END{print $(NF-2)}' /tmp/check_result2)
echo $ota_result
echo  $ota_result >> $RESULT_FILE
 while true; do
   echo "File is empty."
   sleep 1
 done
#grep "OtaResultReport" /tmp/test12 | tail -n 1 | grep -o '"state":[0-9]*' | cut -d':' -f2

# 创建一个临时文件存储接收到的消息


# rostopic echo /my_topic > $TEMP_FILE &
# while true; do
#   echo "File is empty."
#   sleep 1
# done
#gnome-terminal -- bash -c "rostopic echo /ReportApp > $TEMP_FILE; exec bash"
rostopic echo /ReportApp > $TEMP_FILE &
# 启动订阅者，并将输出重定向到临时文件
#rostopic pub --once /my_topic2 std_msgs/String "Hello, ROS!"
while true; do
  if [ $(wc -l < $TEMP_FILE) -eq 0 ]; then
      echo "File is empty."
  else
      content=$(cat $TEMP_FILE)
      echo -n > $TEMP_FILE
      echo "$content"

      echo "File is not empty."
#      states=$($content | grep -o '"state": [01]' | sed 's/"state": //')
#        states=$(echo "$content" | grep -o '"state": [0-1]' | awk '{print $2}')
##      states=$(echo "$content" | grep -o '"state": [01]' | sed 's/"state": //')
      # report=$(jq -r ".OtaResultReport" <<< "$content")
      # ret=$?
      # echo "ret is $ret"
      # if [ $ret -ne 0 ];then
      #   echo "report is empty!!!!!!!!!!!!!!!!!!!!!!!!."
      #   continue
      # fi

      report=$(jq -r ".OtaResultReport" <<< "$content")

      if [ "$report" = "null" ]; then
        echo "report is empty."
        continue
      fi

      # 打印提取的 state 值
      echo "Extracted report values:"
      echo "$report"
      first_value=$(jq -r ".state" <<< "$report")
      echo "get status state values:"
      # first_value=$(echo "$states" | head -n 1)
      echo "$first_value"

      if [ "$first_value" = "" ]; then
        echo "first_value is empty."
        continue
      else
        if [ $first_value -eq 0 ]; then
          echo "first_value == 0"
          echo -e "$first_value" >> $RESULT_FILE
          break
        fi
      fi
      echo -e "$first_value" >> $RESULT_FILE
  fi
  if [ $count -eq 5 ]; then
    echo "time ok ,send ota topic."

    rostopic pub --once /ReqFromApp hj_interface/AppMsg "{appdata: [{key: 'UrlOta', payload: '{"controlLogId":0,"sites":{"fileSize":129342560,"md5":\"491df16d6d7aca92e2437b81ba0fa9a0\","subver":\"V2.0.6.3\","target":1,"type":0,"url":\"https://dev-management-bucket.s3.cn-north-1.amazonaws.com.cn/ota/Scuba_X1_Pro_Max-V2.0.6.3-full-All_1731762485843.bin\"},"version":\"V2.0.6.3\"}', res: 1}], metadata: [], session: '', timestamp: 0, from_: 4, to: 0}"
  fi
  sleep 1
  ((count++))
done

#处理消息结果

while true; do
  echo "get fail ,please check"
  sleep 1
done
