#!/usr/bin/env bash
source /home/robot/data/hj/devel/setup.bash

# Usage:./parse_msg.sh <bagfile>
# This script will extract all topics from the bagfile and save them as individual text files.

topics=`rostopic list  -b $1`

for topic in $topics
do
    echo "Processing topic: $topic"
    rostopic echo -b $1 -p $topic > ${topic:1}.txt
done