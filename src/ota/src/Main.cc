#include <string>
#include <ros/ros.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <ctime>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <atomic>
#include <time.h>
#include <vector>
#include <map>
#include <sys/file.h>
#include "State/BaseState.h"
#include "hjlog.h"
#include "Schedule.h"

void* ota_logger = nullptr;

int main(int argc, char** argv)
{   
    int lockfd = ::open("/tmp/aiper_ota.mark", O_CREAT | O_RDWR, 0666);

    if (lockfd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    if (::flock(lockfd, LOCK_EX | LOCK_NB) < 0) {
        if (errno == EWOULDBLOCK) {
            fprintf(stderr, "Ota Program is running.\n");
            ::close(lockfd);
            exit(EXIT_FAILURE);
        } else {
            perror("flock");
            ::close(lockfd);
            exit(EXIT_FAILURE);
        }
    }

    ros::init(argc, argv, "ota_node", ros::init_options::NoRosout);
    
    ros::NodeHandle n;
    ota_logger = hj_cst_log_add("/userdata/hj/log/ota.log", TRACE_LOG, 1024 * 100, 3);

    aiper_ota::Schedule otaSchedule(n);
    otaSchedule.construct();
    otaSchedule.run();

    return 0;
}