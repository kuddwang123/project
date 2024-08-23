#ifndef TEST_API_H
#define TEST_API_H
#include "function_factory.h"
#include "node_factory.h"
#include "big_data.h"
namespace test_api {
class TestAPI : public hj_bf::Function {
 public:
  explicit TestAPI(const rapidjson::Value &json_conf);
  ~TestAPI(){};
  int loopFunc();
  void callback1(const hj_bf::HJTimerEvent &);
 private:
 hj_bf::HJPublisher test_pub;
 hj_bf::HJPublisher test_pub3;
 hj_bf::HJPublisher test_pub2;
// hj_bf::HJClient client;
 hj_bf::HJTimer timer1;
 int fds_[2];
};
}  // namespace test_load_so

#endif
