#include "test_load_so.h"
#include "log.h"

#include <fcntl.h>
#include <unistd.h>

#include <functional>

#include "collect_node/TwoInts.h"
#include "iostream"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/pointer.h"
#include "rapidjson/writer.h"
#include "std_msgs/String.h"
#include "thread"
#include <unistd.h>
#include <sys/syscall.h>
#include <signal.h>
#include <boost/filesystem.hpp>
HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory"<<FUNCTION_NAME << std::endl;
  factory.registerCreater<test_api::TestAPI>(FUNCTION_NAME);
}

namespace test_api {
int so_func1() {
  static int i = 0;
  i++;
  return i;
}
  void crash() { volatile int* a = (int*)(NULL); *a = 1; }
int TestAPI::loopFunc() {
  std::cerr << "minos in loopFunc" << std::endl;

  collect_node::TwoInts srv;
//  ros::Rate loop_rate(1);
  int count = 0;
  while (1) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "test new framework hello world " << count;
    msg.data = ss.str();

    //    ROS_INFO("%s", msg.data.c_str());
        test_pub.publish(msg);
        test_pub2.publish(msg);
    ++count;

    srv.request.a = 15;
    srv.request.b = 20;
    // if(client.exists()){
    //   if (client.call(srv)) {
    //     ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    //   } else {
    //     ROS_ERROR("Failed to call service add_two_ints");
    //   }
    // } else {
    //   HJ_INFO("minos client not exists");
    //   int ret = 0;
    //   while(!ret){
    //     ret = client.waitForExistence(1000*1000);
    //     HJ_INFO("minos in waitForExistence");
    //   }
    //   HJ_INFO("minos client  exists");
    // }
      if (client.call(srv)) {
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      } else {
        ROS_ERROR("Failed to call service add_two_ints");
      }
/**/
//    write(fds_[1], "n", 1);
    /**/
    usleep(100000);
 //   crash() ;
    HJ_INFO("minos in loopFunc");
    HJ_ERROR("minos in loopFunc");
    static int test_err = 0;
//    std::cerr << "minos test err:" << test_err << std::endl;
//    std::cout << "minos2 test err:" << test_err << std::endl;
    test_err++;
    //
  }
}

void FdcCallback(int pipe_fd) {
  int real_read;
  char buffer[128];
  int buffer_size = sizeof(buffer);
  while (true) {
    real_read = read(pipe_fd, buffer, sizeof(buffer));
    if (real_read < 0) {
      if (errno != EAGAIN) {
        throw std::runtime_error("get the fd error != EAGAIN");
      } else {
        break;
      }
    } else if (!real_read) {
      throw std::runtime_error("the fd should not be close!");
    } else {
      for (int i = 0; i < real_read; i++) {
        if (buffer[i] == 'n') {
          break;
          //        LOCKLOG_2("SpinCallback ", std::to_string(pipe_fd).c_str(), (int64_t)ch);
        }
      }
    }
    if (real_read < buffer_size) {
      break;
    }
  }
  ROS_INFO("FdcCallback triggered");
}
int  signalHandler(int signal)
{
    std::cout << "Caught SIGINT signal." << std::endl;
    exit(signal);
    // 执行自定义操作，如记录日志、终止线程或进程等
}
std::string GetTime() {
  static char nowtime[20];
  time_t rawtime;
  struct tm ltime;
  time(&rawtime);
  localtime_r(&rawtime, &ltime);
  strftime(nowtime, 20, "%Y-%m-%d-%H:%M:%S", &ltime);
  return nowtime;
}

void TestAPI::callback1(const hj_bf::HJTimerEvent &)
{
    // std::cout << "minos22222 so_func:" << test_load_so::so_func() << std::endl;
    static int test_err = 0;
//    std::cerr << "minos test err:" << test_err << std::endl;
//    std::cout << "minos2 test err:" << test_err << std::endl;
    test_err++;
    HJ_ERROR("minos2 in loopFunc %d",test_err);
      // crash();
    int x = 1;
    int y = 1;
//    HJ_CHECK_EQ3(1, 2, "my test1");
    std::string sx = "tests";
    std::string sy = "tests";
//    HJ_CHECK_EQ3(sx, sy, 1111);

  std::string test_json = R"({
        "ID": "core_dump",
        "name": "core_dump",
        "dump_path": "/tmp/dump_collect_node"
      })";
      // std::string src_name = "/userdata/test_big_data.zip";
      // std::string dest_name = src_name + GetTime();
      // if (boost::filesystem::exists(src_name)) {
      //   boost::filesystem::copy_file(src_name, dest_name);
      //   //       big_data::InsertBigdata(test_json,"/userdata/test_big_data.zip",
      //   //       big_data::kBigdataImmediate|big_data::kBigdataPack);
        big_data::InsertBigdata(test_json);
}

bool add2(collect_node::TwoInts::Request& req, collect_node::TwoInts::Response& res) {
  res.sum = req.a + req.b + 1;
  ROS_INFO("request2: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending add2 response: [%ld]", (long int)res.sum);
  return true;
}

TestAPI::TestAPI(const rapidjson::Value& json_conf) : hj_bf::Function(json_conf) {

  if (json_conf.HasMember("test_transmit") && json_conf["test_transmit"].IsString()) {
    std::string val_str = json_conf["test_transmit"].GetString();
    std::cerr << "minos get transmit:" << val_str << std::endl;
  }

  // blackboard = BT::Blackboard::create<ancp::StaticBlackboardRobinMap>();
//  HJ_INFO("minos test %d",10);
//  HJ_INFO("minos test" << 10);
//  HJ_ERROR("minos233 test %s",ROSCONSOLE_DEFAULT_NAME);
//  HJ_INFO("minos233 test" << ROSCONSOLE_DEFAULT_NAME);

//  auto test_pub_test = hj_bf::HJAdvertise<std_msgs::String>("collect_testfffffffff", 1000);
  test_pub = hj_bf::HJAdvertise<std_msgs::String>("collect_test2", 1000,true);
  test_pub2 = hj_bf::HJAdvertise<std_msgs::String>("collect_test3", 1000, true);
  client = hj_bf::HJCreateClient<collect_node::TwoInts>("add_two_ints");
  // auto state = std::thread(&TestAPI::loopFunc, this);  // 开线程
  // state.detach();
  // timer1 = hj_bf::HJCreateTimer("timerloop", 1*1000 * 1000, &TestAPI::callback1, this);
  // test fd
//  crash();
  if (pipe(fds_) == -1) {
    exit(EXIT_FAILURE);
  }
  int flag = fcntl(fds_[1], F_GETFL);
  flag |= O_NONBLOCK;
  fcntl(fds_[1], F_SETFL, flag);

  flag = fcntl(fds_[0], F_GETFL);
  flag |= O_NONBLOCK;
  fcntl(fds_[0], F_SETFL, flag);
  hj_bf::registerFdCtrl(fds_[0], std::bind(&FdcCallback, std::placeholders::_1));
  test_service1_ = hj_bf::HJCreateServer("client1_test", add2);
/*
  struct sigaction sa;
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGTERM, &sa, nullptr);
  */
/**/
  /*  */
}
}  // namespace test_api
