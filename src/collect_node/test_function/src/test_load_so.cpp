#include "test_function.h"
#include "log.h"
#include <unistd.h>

#include "collect_node/TwoInts.h"
#include "iostream"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"
#include "rapidjson/writer.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "thread"
#include <functional>
#include "shm_interface.h"
#include <hj_interface/BigdataUpload.h>
// #include "test_load_so.h"
HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" <<FUNCTION_NAME<< std::endl;
  factory.registerCreater<test_load_so::TestLoadSo>(FUNCTION_NAME);
}

namespace test_load_so {
int so_func() {
  static int i = 0;
  i++;
  return i;
}
void crash() { volatile int* a = (int*)(NULL); *a = 1; }
class TestClassFun {
 public:
  void chatterCallback(const std_msgs::String::ConstPtr& msg) { 
    ROS_INFO("I heard: [%s] in class 10", msg->data.c_str()); 
    sleep(1);
    ROS_INFO("I heard: [%s] in class 11", msg->data.c_str());
    sleep(1);
    ROS_INFO("I heard: [%s] in class 12", msg->data.c_str());
    sleep(1);
    }
  void chatterCallback2(const std_msgs::String::ConstPtr& msg) { 
    ROS_INFO("I heard: [%s] in class 20", msg->data.c_str()); 
    }
};

void chatterCallback(const std_msgs::String::ConstPtr& msg) { ROS_INFO("I heard: [%s]", msg->data.c_str()); }

bool add(collect_node::TwoInts::Request& req, collect_node::TwoInts::Response& res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending back response: [%ld]", (long int)res.sum);
  return true;
}
bool add2(collect_node::TwoInts::Request& req, collect_node::TwoInts::Response& res) {
  res.sum = req.a + req.b + 1;
  ROS_INFO("request2: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending 2back response: [%ld]", (long int)res.sum);
  return true;
}
// int  loopFunc(){
//   while (1) {
//     {
//       hj_bf::MinosLock my_lock("my_lock");
//       std::cout << "minos1:"
//                 << "4, my_lock in" << std::endl;
//       sleep(1);
//       std::cout << "minos1:"
//                 << "4, my_lock out" << std::endl;
//     }
//     sleep(1);
//     hj_bf::MinosLock my_lock2("my_lock");
//     std::cout << "minos1:"
//               << "42, my_lock in" << std::endl;
//     sleep(1);
//     std::cout << "minos1:"
//               << "42, my_lock out" << std::endl;
//   }
// }
void TestLoadSo::callback1(const hj_bf::HJTimerEvent &)
{
  // static int num = 0;
  // int temp;
  // const int temp_const = 1;
  // int& temp_yy = temp;
  // double test = ros::Time::now().toSec();
  // HJ_INFO("MINOS1:%lf",test);
  // HJ_INFO("MINOS2:%lf",ros::Time().fromSec(test).toSec());
  // // std::cout << "minos2 so_func1:" << test_api::so_func1() << std::endl;
  // // HJ_INFO("callback1 typeinfo 1:%s",typeid(temp_yy).name());
  // // HJ_INFO("callback1 typeinfo 1:%s",typeid(temp).name());
  // // HJ_INFO("callback1 typeinfo 1:%s",typeid(temp_const).name());
  // double int_start = ros::Time::now().toSec();
  // double int_end = ros::Time::now().toSec();
  // int_start = ros::Time::now().toSec();
  // hj_bf::setVariable("minos_test1", num);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos setVariable:%f",int_end - int_start);
  // int_start = ros::Time::now().toSec();
  // hj_bf::getVariable("minos_test1", temp);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos getVariable:%f",int_end - int_start);
  // HJ_INFO("callback1 getVariable:%d",temp);

  // double temp_set_double = 0.123;
  // temp_set_double += num;
  // double temp_get_double;
  // int_start = ros::Time::now().toSec();
  // hj_bf::setVariable("minos_test_double", temp_set_double);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos setVariable double:%f",int_end - int_start);
  // int_start = ros::Time::now().toSec();
  // hj_bf::getVariable("minos_test_double", temp_get_double);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos getVariable double:%f",int_end - int_start);
  // HJ_INFO("callback1 getVariable double:%f",temp_get_double);

  // num ++;

  // std::string test_set_array;
  // char set_array[50] = "this si a test array, test num is ?";
  // test_set_array = set_array;
  // char get_array[50];
  // test_set_array += std::to_string(num);
  // memcpy(set_array, test_set_array.c_str(), test_set_array.size());
  // HJ_INFO("callback1 typeinfo 2:%s",typeid(get_array).name());
  // int_start = ros::Time::now().toSec();
  // hj_bf::setVariable("minos_test_array", set_array);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos setVariable array:%f",int_end - int_start);
  // int_start = ros::Time::now().toSec();
  // hj_bf::getVariable("minos_test_array", get_array);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos getVariable array:%f",int_end - int_start);

  // HJ_INFO("callback1 getVariable array:%s",get_array);

  // int set_int_array[10] = {1,2,3,4,5,6,7,8,9,10};
  // int get_int_array[10];
  // set_int_array[9] += num;
  // HJ_INFO("callback1 typeinfo 3:%s",typeid(set_int_array).name());
  // int_start = ros::Time::now().toSec();
  // hj_bf::setVariable("minos_test_int_array", set_int_array);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos setVariable int array:%f",int_end - int_start);
  // int_start = ros::Time::now().toSec();
  // hj_bf::getVariable("minos_test_int_array", get_int_array);
  // int_end = ros::Time::now().toSec();
  // HJ_INFO("minos getVariable int array:%f",int_end - int_start);
  // HJ_INFO("callback1 getVariable int array:%d %d %d %d %d %d %d %d %d %d ", get_int_array[0], get_int_array[1],
  //         get_int_array[2], get_int_array[3], get_int_array[4], get_int_array[5], get_int_array[6], get_int_array[7],
  //         get_int_array[8], get_int_array[9]);

  //   HJ_INFO("Callback size1:%d",sizeof(boost::interprocess::interprocess_mutex));
  //   HJ_INFO("Callback size1:%d",sizeof(hj_bf::VariableInstance<long>));
  //   HJ_INFO("Callback size1:%d",sizeof(int));
  //   HJ_INFO("Callback size1:%d",sizeof(hj_bf::VariableImpl));

  // std_msgs::Int32 temp2;
  // temp2.data = 32;
  // test_pub_.publish(temp2);

  // std_msgs::String temp_msg;
  // temp_msg.data = "test pub";
  // test_pub2_.publish(temp_msg);

  // static int test_flag = 0;
  // static int test2_flag = 0;
  // if (test_flag > 10) {
  //   if (test2_flag == 0) {
  //     test_service_ = hj_bf::HJCreateServer("add_two_ints", add2);
  //   }
  //   test2_flag = 1;
  // }
  // test_flag ++;
  std::string test_string = "test_string";
  HJ_INFO("test  HJ_INFO:%d, %f, %s", 1, 1.1, "test");
  HJ_INFO_STREAM("test HJ_INFO_STREAM:"<< " string " << 1 << 1.11);
  HJ_WARN_STREAM("test HJ_WARN_STREAM:"<< " string " << test_string.c_str());
  HJ_ERROR_STREAM("test HJ_ERROR_STREAM:"<< " string " << test_string);
  HJ_EFATAL_STREAM("test HJ_EFATAL_STREAM:"<< " string " );
  HJ_IMPORTANT_STREAM("test HJ_IMPORTANT_STREAM:"<< " string " );

  HJ_INFO("HJ_INFO :");
  HJ_WARN("HJ_WARN :");
  HJ_ERROR("HJ_ERROR :");
  HJ_EFATAL("HJ_EFATAL :");

  // collect_node::TwoInts srv;
  // static int i = 0;
  // i ++;
  // srv.request.a = i;
  // srv.request.b = 20;
  // if (client1.call(srv)) {
  //   HJ_INFO("callback1 Sum: %ld", (long int)srv.response.sum);
  // } else {
  //   HJ_ERROR("Failed to call service callback1");
  // }
  // char* switch_cstr = NULL;
  // static int ff = 1;
  // switch_cstr = getenv("test_flag");
  // if(switch_cstr == NULL) {
  //   HJ_ERROR("switch_cstr == NULL");
  // }else{
  //   HJ_ERROR("switch_cstr =%s",switch_cstr);
  // }
}
void TestLoadSo::callback1_for_steady(const hj_bf::HJSteadyTimerEvent &)
{
  collect_node::TwoInts srv;
  static int i = 100;
  i ++;
  srv.request.a = i;
  srv.request.b = 20;
  if (client1.call(srv)) {
    ROS_INFO("callback1_for_steady Sum: %ld", (long int)srv.response.sum);
  } else {
    ROS_ERROR("Failed to call service callback1_for_steady");
  }
}
void TestLoadSo::callbackNormal()
{
  HJ_INFO("callbackNormal 1 triggered time:%lf",ros::Time::now().toSec());
  HJ_INFO("callbackNormal :%d",test_index_);
  test_index_++;
}
void TestLoadSo::callback2(const hj_bf::HJTimerEvent &)
{
  // ROS_INFO("Callback 20 triggered");
  // sleep(1);
  // static bool flag = false;
  // if(flag == false){
  //   flag = true;
  //   ROS_INFO("timer3 start");
  //   timer3.start();
  // }
  // ROS_INFO("Callback 21 triggered");
  // sleep(1);
  // ROS_INFO("Callback 22 triggered");
  // sleep(1);
  std::string test_json2 = R"({
        "ID": "log_redirect",
        "name": "log_redirect",
        "err_log_file_path": "2",
        "cout_log_file_path": "2"
      })";
  hj_interface::BigdataUpload temp;
  temp.payload = test_json2;
  temp.upload_file = "";
  temp.type = big_data::BigdataCmd::kBigdataImmediate;
  bigdata_pub_.publish(temp);
}
void callback3(const hj_bf::HJTimerEvent &)
{
  ROS_INFO("Callback 30 triggered");
  std::string test_json = R"({
        "ID": "log_redirect",
        "name": "log_redirect",
        "err_log_file_path": "",
        "cout_log_file_path": ""
      })";
      
  big_data::InsertBigdata(test_json);

  // sleep(1);
  // ROS_INFO("Callback 31 triggered");
  // sleep(1);
  // ROS_INFO("Callback 32 triggered");
  // sleep(1);
  // test_pub_.publish("test new pub");
}

TestLoadSo::~TestLoadSo(){
//    timer1.stop();
//    timer2.stop();
//temp_timer.stop();
    std::cerr<<"your function exit"<<std::endl;
//        timer.stop();
    };


int TestLoadSo::loopPrint() {
  int test_count = 0;
  while (1) {
    sleep(1);
//    std::cerr << "minos getNumPublishers:" << sub.getNumPublishers() << std::endl;
    std::cerr << "minos test count=" << test_count << std::endl;
    test_count++;
    collect_node::TwoInts srv;
    static int i = 0;
    i ++;
    srv.request.a = i;
    srv.request.b = 20;
    if (client1.call(srv)) {
      ROS_INFO("loopPrint Sum: %ld", (long int)srv.response.sum);
    } else {
      ROS_ERROR("Failed to call service loopPrint");
    }
  }
}

// int loopPrint2() {

//   hj_bf::MinosCondition my_condition("test_condition");
//   while (1) {
//     {
//       std::cout << "minos1:"
//                 << "4, before wait" << std::endl;
//         hj_bf::MinosLock my_lock("test_condition_lock");
//         my_condition.wait(my_lock);

//       std::cout << "minos1:"
//                 << "4, in wait" << std::endl;
//     }
//   }
// }

TestLoadSo::TestLoadSo(const rapidjson::Value& json_conf) : hj_bf::Function(json_conf) {
  //read config
  if (json_conf.HasMember("test_transmit") && json_conf["test_transmit"].IsString()) {
    std::string val_str = json_conf["test_transmit"].GetString();
    std::cerr << "minos get transmit:" << val_str << std::endl;
  }
  ROS_INFO("minos here in TestLoadSo:%s",__FILE__);

  std::function<void(void)> sub_callback =
      std::bind(&TestLoadSo::callbackNormal, this);

    // 定义定时器的回调函数
    auto callback = []() {
        // 在这里执行定时器的操作
        HJ_INFO("new time:%lf",ros::Time::now().toSec());
    };
    // 启动定时器，周期为1秒
    // timer_h_.start(1, sub_callback);
    // 程序运行10秒
  ROS_INFO("minos out here in TestLoadSo");
  std::cerr << "minos out here in TestLoadSo" << std::endl;

  //your code
  TestClassFun test_class_fun;//for test fun in class
  test_pub_ = hj_bf::HJAdvertise<std_msgs::Int32>("collect_test333", 1000);
  test_pub2_ = hj_bf::HJAdvertise<std_msgs::String>("collect_test3", 1000);
  bigdata_pub_ = hj_bf::HJAdvertise<hj_interface::BigdataUpload>(big_data::kBigdataCmdTopicName, 1000);
//  HJ_INFO("getNumPublisher:%d", hj_bf::getNumPublisher("collect_test3"));
//  HJ_INFO("getNuAllPublisher:%d", hj_bf::getAllPublisher());
  boost::function<void(const std_msgs::String::ConstPtr&)> func_callback(chatterCallback, boost::placeholders::_1);
//  hj_bf::HJSubscriber sub = hj_bf::HJSubscribe("/collect_test2", 1000, &TestClassFun::chatterCallback, &test_class_fun);
   test_sub_ = hj_bf::HJSubscribe("collect_test2", 1000, func_callback);
//  test_sub2_ = hj_bf::HJSubscribe("collect_test", 1000, &TestLoadSo::chatterCallbacktt, this);
//  test_sub2_ = hj_bf::HJSubscribe("collect_test", 1000, [](const std_msgs::String::ConstPtr& msg){ROS_INFO("minos here in 2222222");});
//  hj_bf::getHandle().subscribe<std_msgs::String>("collect_test", 1000, [](const std_msgs::String::ConstPtr& msg){ROS_INFO("minos here in 2222222");});

//  hj_bf::HJServer service = test_service_;
//  hj_bf::HJServer service2 = service;
//test timer
//  temp_ = hj_bf::temp_test;


//  static std::shared_ptr<ros::Timer> temp_timer2 =  std::make_shared<ros::Timer>(hj_bf::getHandle().createTimer(ros::Duration(10), &TestLoadSo::callback1, this));
//  static std::weak_ptr<ros::Timer> temp_timer;
//  temp_timer = temp_timer2;
//  temp_timer->stop();
//  std::cerr << "minos TestLoadSo:" << temp_timer2.use_count() << std::endl;
    int num = 0;
  //  hj_bf::createVariable("minos_test1", num);
   timer1 = hj_bf::HJCreateTimer("timer1",  1000 * 1000, &TestLoadSo::callback1, this);
   steadytimer1 = hj_bf::HJCreateSteadyTimer("steadytimer1",  1000 * 1000, &TestLoadSo::callback1_for_steady, this);

   client1 = hj_bf::HJCreateClient<collect_node::TwoInts>("client1_test");
   client2 = hj_bf::HJCreateClient<collect_node::TwoInts>("client2_test");
  // ROS_INFO("minos getNumTimer:%d",hj_bf::getNumTimer());
   
  // timer1.stop();
  // sleep(1);
//   timer_h_.start();
  // timer2 = hj_bf::HJCreateTimer("timer2", 2 * 1000 * 1000, &TestLoadSo::callback2, this);
  // timer3 = hj_bf::HJCreateTimer("timer3", 2 * 1000* 1000, callback3);
//  timer3.stop();
//  auto state = std::thread(&TestLoadSo::loopPrint,this);  // 开线程
//   state.detach();
//  auto state1 = std::thread(&TestLoadSo::loopPrint,this);  // 开线程
//   state1.detach();
//  auto state2 = std::thread(&TestLoadSo::loopPrint,this);  // 开线程
//   state2.detach();
//  auto state3 = std::thread(&TestLoadSo::loopPrint,this);  // 开线程
//   state3.detach();
//  auto state4 = std::thread(&TestLoadSo::loopPrint,this);  // 开线程
//   state4.detach();
    char *heap_buf = (char*)malloc(32*sizeof(char));
    memcpy(heap_buf+30, "overflow", 8);    //在heap_buf的第30个字节开始，拷贝8个字符

    free(heap_buf);
}

  void TestLoadSo::chatterCallbacktt(const std_msgs::String& msg) { 
    ROS_INFO("I heard: [%s] in class 10", msg.data.c_str()); 
    // sleep(1);
    // ROS_INFO("I heard: [%s] in class 11", msg.data.c_str());
    // sleep(1);
    // ROS_INFO("I heard: [%s] in class 12", msg.data.c_str());
    // sleep(1);
    }
    void TestLoadSo::chatterCallbacktt2(const std_msgs::String::ConstPtr&  msg) { 
      ROS_INFO("I heard: [%s] in class 10", msg->data.c_str()); 
      sleep(1);
      ROS_INFO("I heard: [%s] in class 11", msg->data.c_str());
      sleep(1);
      ROS_INFO("I heard: [%s] in class 12", msg->data.c_str());
      sleep(1);
      }
}  // namespace test_load_so
