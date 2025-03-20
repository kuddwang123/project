#include <string>
#include <ros/ros.h>
#include <fcntl.h>
#include <stdio.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <sys/file.h>
#include "Schedule.h"


int main(int argc, char** argv)
{   
    int lockfd = ::open("/tmp/fw_cut.mark", O_CREAT | O_RDWR, 0666);

    if (lockfd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    if (::flock(lockfd, LOCK_EX | LOCK_NB) < 0) {
        if (errno == EWOULDBLOCK) {
            fprintf(stderr, "another fw_cut is running.\n");
            ::close(lockfd);
            exit(EXIT_FAILURE);
        } else {
            perror("flock");
            ::close(lockfd);
            exit(EXIT_FAILURE);
        }
    }

    if (!aiper_ota::utils::isFileExist("/userdata/ota/ota.bin")) {
        fprintf(stderr, "no ota.bin found in /userdata/ota/\n");
        exit(EXIT_FAILURE);
    }

    ros::init(argc, argv, "fwcut_node", ros::init_options::NoRosout);
    ros::NodeHandle n;

    aiper_ota::Schedule otaSchedule(n);
    otaSchedule.construct();
    otaSchedule.run();
    

    return 0;
}