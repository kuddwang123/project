#ifndef TEST_FUNCTION_H
#define TEST_FUNCTION_H
#include "function_factory.h"
#include "node_factory.h"
#include "std_msgs/String.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include "big_data.h"
#include "blackboard/blackboard_robin_map.h"
#include "blackboard/blackboard_local.h"
namespace test_load_so {
int so_func();
class TestLoadSo : public hj_bf::Function {
 public:
  explicit TestLoadSo(const rapidjson::Value &json_conf);
  ~TestLoadSo();
  void callback1(const hj_bf::HJTimerEvent &);
  void callback2(const hj_bf::HJTimerEvent &);
  void callback1_for_steady(const hj_bf::HJSteadyTimerEvent &);
  void callbackNormal();
  int loopPrint();
  void loopPrint1();
  void loopPrint2();
 private:
  hj_bf::HJPublisher test_pub_;
  hj_bf::HJPublisher test_pub2_;
  hj_bf::HJPublisher bigdata_pub_;
  hj_bf::HJSubscriber test_sub_;
  hj_bf::HJSubscriber test_sub2_;
  void chatterCallbacktt(const std_msgs::String& msg);
  void chatterCallbacktt2(const std_msgs::String::ConstPtr&  msg);
//  std::shared_ptr<ros::Timer> temp_timer;
//  std::shared_ptr<ros::Timer> temp_timer2;
  hj_bf::HJServer test_service_;
//  hj_bf::HJTimer timer1;
  hj_bf::HJTimer timer1;
  hj_bf::HJTimer timer3;
  hj_bf::HJTimer timer2;
  hj_bf::HJTimer timer4;
  hj_bf::HJSteadyTimer steadytimer1;
  hj_bf::HighResolutionTimer timer_h_;
  hj_bf::HJClient client1;
  hj_bf::HJClient client2;
  std::mutex pub_mutex_;
  int test_index_{0};
};
}  // namespace test_load_so

#endif
