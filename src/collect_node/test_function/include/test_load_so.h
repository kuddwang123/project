#ifndef TEST_LOAD_SO_H
#define TEST_LOAD_SO_H
#include "function_factory.h"
#include "node_factory.h"
#include "std_msgs/String.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include "big_data.h"
namespace test_load_so {

class TestLoadSo : public hj_bf::Function {
 public:
  explicit TestLoadSo(const rapidjson::Value &json_conf);
  ~TestLoadSo();
  void callback1(const hj_bf::HJTimerEvent &);
  void callback2(const hj_bf::HJTimerEvent &);
  void callbackNormal();
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
  hj_bf::HighResolutionTimer timer_h_;
  
  int test_index_{0};
};
}  // namespace test_load_so

#endif
