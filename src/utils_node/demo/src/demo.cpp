// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "demo.h"
#include "log.h"
#include "std_msgs/String.h"
#include "shm_interface.h"
#include "big_data.h"
#include "thread"
HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<utils_node_ns::Demo>(FUNCTION_NAME);
}
namespace utils_node_ns {

// bool checkPublisherExists(const std::string& topic_name)
// {
//     ros::master::V_TopicInfo topics;
//     ros::master::getTopics(topics);

//     for (const auto& topic : topics)
//     {
//         if (topic.name == topic_name && topic.publisher_count > 0)
//         {
//             return true;
//         }
//     }
//     return false;
// }

void Demo::callback2(const hj_bf::HJTimerEvent &)
{
  int temp;
  double int_start = ros::Time::now().toSec();
  double int_end = ros::Time::now().toSec();
  int ret = 0;
  int_start = ros::Time::now().toSec();
  ret = hj_bf::getVariable("minos_test1", temp);
  int_end = ros::Time::now().toSec();
  HJ_INFO("minos getVariable:%f",int_end - int_start);
  if(ret){
    HJ_INFO("callback2 getVariable:%d",temp);
  }else{
    HJ_INFO("callback2 getVariable fail");
  }
  double temp_double;
  int_start = ros::Time::now().toSec();
  ret = hj_bf::getVariable("minos_test_double", temp_double);
  int_end = ros::Time::now().toSec();
  HJ_INFO("minos getVariable double :%f",int_end - int_start);
  if(ret){
    HJ_INFO("callback2 getVariable double :%f",temp_double);
  }else{
    HJ_INFO("callback2 getVariable double fail");
  }

  std::string test_set_array;

  char get_array[50];
  int_start = ros::Time::now().toSec();
  ret = hj_bf::getVariable("minos_test_array", get_array);
  int_end = ros::Time::now().toSec();
  HJ_INFO("minos getVariable array :%f",int_end - int_start);
  if(ret){
    HJ_INFO("callback1 getVariable array:%s",get_array);
  } else {
      HJ_INFO("callback2 getVariable array fail");
  }
  // hj_bf::getVariable("minos_test_array", get_array);
  
  int get_int_array[10];
  int_start = ros::Time::now().toSec();
  ret = hj_bf::getVariable("minos_test_int_array", get_int_array);
  int_end = ros::Time::now().toSec();
  HJ_INFO("minos getVariable int array :%f",int_end - int_start);
  if(ret ){
    HJ_INFO("callback1 getVariable int array:%d %d %d %d %d %d %d %d %d %d ", get_int_array[0], get_int_array[1],
            get_int_array[2], get_int_array[3], get_int_array[4], get_int_array[5], get_int_array[6], get_int_array[7],
            get_int_array[8], get_int_array[9]);
  } else {
      HJ_INFO("callback2 getVariable int array fail");
  }

  {
    hj_bf::MinosLock my_lock("my_lock");
    std::cout << "minos1:"
              << "3, my_lock in" << std::endl;
    sleep(10);
    std::cout << "minos1:"
              << "3, my_lock out" << std::endl;
  }
}

// int  loopFunc() {
//   while (1) {
//     {
//       hj_bf::MinosLock my_lock("my_lock");
//       std::cout << "minos1:"
//                 << "3, my_lock in" << std::endl;
//       sleep(10);
//       std::cout << "minos1:"
//                 << "3, my_lock out" << std::endl;
//     }
//   }
// }
// int  loopFunc2() {
//   hj_bf::MinosCondition my_condition("test_condition");
//   while (1) {
//     {
//       sleep(5);
//       std::cout << "minos1:"
//                 << "before notify" << std::endl;
//       my_condition.notify_one();
//       std::cout << "minos1:"
//                 << "3, notify success" << std::endl;
//     }
//   }
// }

void callback3(const hj_bf::HJTimerEvent &)
{
  ROS_INFO("Callback util demo triggered");
  std::string test_json = R"({
        "ID": "log_redirect",
        "name": "log_redirect",
        "err_log_file_path": "utils demo",
        "cout_log_file_path": "utils demo"
      })";
      
  big_data::InsertBigdata(test_json);

  // sleep(1);
  // ROS_INFO("Callback 31 triggered");
  // sleep(1);
  // ROS_INFO("Callback 32 triggered");
  // sleep(1);
  // test_pub_.publish("test new pub");
}

void chatterCallback(const std_msgs::String::ConstPtr& msg) { ROS_INFO("I heard in demo : [%s]", msg->data.c_str()); }

Demo::Demo(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  boost::function<void(const std_msgs::String::ConstPtr&)> func_callback(chatterCallback, boost::placeholders::_1);

  test_sub_ = hj_bf::HJSubscribe("collect_test", 1000, func_callback);
  // your code
  std::cerr << "minos just a demo" << std::endl;
  timer2 = hj_bf::HJCreateTimer("timer2", 3* 1000* 1000, &Demo::callback2,this);
//  auto state = std::thread(&loopFunc2);  // 开线程
  // state.detach();
}
}  // namespace utils_node_ns
