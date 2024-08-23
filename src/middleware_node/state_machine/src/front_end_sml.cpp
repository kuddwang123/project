// @file front_end_sml.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#include "front_end_sml.h"

#include "front_end_data.h"
#include "log.h"
namespace hj_sml {
namespace sml = boost::sml;
sml::sm<HjSml, sml::thread_safe<std::recursive_mutex>>& GetSmlInstance() {  // inline
  static HjSml sml_class{};
  static sml::sm<HjSml, sml::thread_safe<std::recursive_mutex>> sm{sml_class};
  //  static std::shared_ptr<sml::sm<HjSml>> sm_ptr = std::make_shared<sml::sm<HjSml>>({});
  return sm;
}
std::string idle_string = "hj_sml::idle";
std::string s1_string = "hj_sml::s1";
std::string s2_string = "hj_sml::s2";
std::string s3_string = "hj_sml::s3";
std::string ota_string = "hj_sml::ota_s";

void HjSml::emptyFunc() {}
void print_state_event(std::string s, std::string e){
  std::cout << "s: " << s<< " e:" <<e<< std::endl;
}

bool HjSml::guard1() {
  print_state_event(GET_STATE_EVENT_NAME(idle), GET_STATE_EVENT_NAME(e1));
  std::cout << "guard1: " << std::endl;
  sml_num_++;
  return true;
};
void HjSml::on_entry1() {
  std::cout << "on_entry1: " << sml_num_ << std::endl;
  sml_num_++;
};
void HjSml::test_pre_action() {
  std::cout << "test_pre_action: " << sml_num_ << std::endl;
  hj_sml::GetSmlInstance().visit_current_states([](auto state) {
    previous_state = state.c_str();
    HJ_INFO("test_pre_action state:%s",previous_state.c_str());
  });
};

bool HjSml::on_entrytest() {
  std::cout << "on_entrytest: " << sml_num_ << std::endl;
  sml_num_++;
  return false;
};
void global_test_pre_action() {
  std::cout << "test_pre_action: " << std::endl;
  hj_sml::GetSmlInstance().visit_current_states([](auto state) {
    previous_state = state.c_str();
    HJ_INFO("test_pre_action state:%s",previous_state.c_str());
  });
};

std::function<void()> test_function(global_test_pre_action);
void HjSml::on_exit1() {
//  std::cout << "on_exit1: " << sml_num_ << std::endl;
  sml_num_++;
};
void HjSml::checkS1() {
  if ((previous_state == s1_string) || (previous_state == s2_string)) {
    std::cerr << "minos checkS1 err previous_state:" << previous_state << std::endl;
  }
}
void HjSml::checkS2() {
  if ((previous_state != s1_string)) {
    std::cerr << "minos checkS2 err previous_state:" << previous_state << std::endl;
  }
}
void HjSml::checkS3() {
  if ((previous_state != s2_string)) {
    std::cerr << "minos checkS3 err previous_state:" << previous_state << std::endl;
  }
}
void HjSml::checkOTA() {
  if ((previous_state == idle_string) || (previous_state == ota_string)) {
    std::cerr << "minos checkS3 err previous_state:" << previous_state << std::endl;
  }
}
// func def
HJ_CALLBACK_INSTANCE(action2) {
  // std::cout << "action2" << std::endl;
}
HJ_GUARD_CALLBACK_INSTANCE(guard3) {
//  std::cout << "guard3" << std::endl;
  return true;
}

HJ_PRE_ACTION_CALLBACK_INSTANCE(test_pre_action_s) {
  std::cout << "test_pre_action_s" << std::endl;
}

void action1::operator()(){
    // std::cout << "action1" << std::endl;
};
bool guard2::operator()() {
  std::cout << "guard2" << std::endl;

  return false;
};
bool HjSml::SetPreviousState::operator()() {
  hj_sml::GetSmlInstance().visit_current_states([](auto state) {
    previous_state = state.c_str();
    HJ_INFO("previous state:%s",previous_state.c_str());
  });
  return true;
};

bool HjSml::setPreviousState() {
  hj_sml::GetSmlInstance().visit_current_states([](auto state) {
    previous_state = state.c_str();
    HJ_INFO("previous state:%s",previous_state.c_str());
  });

  return true;
}

}  // namespace hj_sml
