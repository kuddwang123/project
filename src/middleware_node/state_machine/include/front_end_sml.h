// @file front_end_sml.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-1-19)
#ifndef HJ_SML_H
#define HJ_SML_H
#include <functional>
#include <memory>
#include <mutex>

#include "base_sml.h"
#include "demo_state.h"
#include "front_end_data.h"
#include "ota_state.h"
namespace hj_sml {
namespace sml = boost::sml;
extern std::function<void()> test_function;
#define MIDDLEWARE_BEGIN_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  HJ_TRANSITION_BEGIN_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, action_func, state_dst)

#define MIDDLEWARE_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  HJ_TRANSITION_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, action_func, state_dst)

#define MIDDLEWARE_END_ITEM(state_src, event_name, guard_func, action_func) \
  HJ_TRANSITION_END_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, action_func)

#define MIDDLEWARE_CLASS_BEGIN_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  HJ_TRANSITION_CLASS_BEGIN_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, action_func, state_dst)

#define MIDDLEWARE_CLASS_ITEM(state_src, event_name, guard_func, action_func, state_dst) \
  HJ_TRANSITION_CLASS_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, action_func, state_dst)

#define MIDDLEWARE_CLASS_END_ITEM(state_src, event_name, guard_func, action_func) \
  HJ_TRANSITION_CLASS_END_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, action_func)

#define MIDDLEWARE_CLASS_PRE_ACTION_ITEM(state_src, event_name, guard_func, pre_cation, action_func, state_dst) \
  HJ_TRANSITION_CLASS_PRE_ACTION_ITEM(state_src, event_name, (guard_func) && SetPreviousState{}, pre_cation, action_func, state_dst)
auto on_entry3 = []() { std::cout << "on_entry3" << std::endl; };
HJ_CALLBACK_DEFINE(action2)
HJ_GUARD_CALLBACK_DEFINE(guard3)
HJ_PRE_ACTION_CALLBACK_DEFINE(test_pre_action_s);
struct action1 {
  void operator()();
};
struct guard2 {
  bool operator()();
};
enum HJSmlState {
  STATE_IDLE = 0,
  STATE_S1,
  STATE_S2,
  STATE_S3,
  STATE_OTA
};

enum HJSmlEvent {
  EVENT_E1 = 0,
  EVENT_E2,
  EVENT_E3,
  EVENT_E4,
  EVENT_OTA,
  EVENT_OTA_OUT
};

HJ_STATE(idle)
HJ_STATE(s1)
HJ_STATE(s2)
HJ_STATE(s3)
HJ_STATE(ota_s)

HJ_EVENT(e1)
HJ_EVENT(e2)
HJ_EVENT(e3)
HJ_EVENT(e4)
HJ_EVENT(ota)
HJ_EVENT(ota_out)

std::string getCurrentState();
class HjSml {
  using Self = HjSml;

 public:
  void emptyFunc();
  void on_entry1();
  void test_pre_action();
  bool on_entrytest();
  void on_exit1();
  bool guard1();
  void checkS1();
  void checkS2();
  void checkS3();
  void checkOTA();
  struct SetPreviousState {
    bool operator()();
  };
  bool setPreviousState();
  HJ_SML_OPERATOR(
  HJ_TRANSITION_TABLE(
    MIDDLEWARE_CLASS_BEGIN_ITEM(idle, e1, &Self::guard1 , &Self::on_entry1, s1)
//    MIDDLEWARE_CLASS_ITEM(s1, e2, guard2{}, demo_state::DemoStateEntry1{}, s2)
    MIDDLEWARE_CLASS_PRE_ACTION_ITEM(s1, e2, guard2{}, test_function, demo_state::DemoStateEntry1{}, s2)
    MIDDLEWARE_CLASS_ITEM(s2, e2, guard2{}, demo_state::DemoStateEntry2{}, s3)
    MIDDLEWARE_CLASS_ITEM(s3, e2, guard2{}, demo_state::DemoStateEntry2{}, s1)
    MIDDLEWARE_CLASS_ITEM(s1, ota, guard2{}, &Self::on_entrytest, ota_s)
    MIDDLEWARE_CLASS_ITEM(s2, ota, guard3{}, ota_state::OtaStateEntry1{}, ota_s)
    MIDDLEWARE_CLASS_ITEM(s3, ota, HJ_EMPTY_GUARD(), ota_state::OtaStateEntry1{}, ota_s)
    MIDDLEWARE_CLASS_ITEM(ota_s, ota_out, HJ_EMPTY_GUARD(), ota_state::OtaStateEntry2{}, s1)

    HJ_TRANSITION_CLASS_ENTRY_ITEM(s1, &Self::checkS1)
    HJ_TRANSITION_CLASS_ENTRY_ITEM(s2, &Self::checkS2)
    HJ_TRANSITION_CLASS_ENTRY_ITEM(s3, &Self::checkS3)
    HJ_TRANSITION_CLASS_ENTRY_ITEM(ota_s, &Self::checkOTA)
    HJ_TRANSITION_CLASS_EXIT_ITEM(s1, demo_state::DemoStateExit1{})
    HJ_TRANSITION_CLASS_EXIT_ITEM(s2, demo_state::DemoStateExit2{})
    HJ_TRANSITION_CLASS_EXIT_ITEM(ota_s, ota_state::OtaStateExit1{})
    MIDDLEWARE_CLASS_END_ITEM(s1, e4, guard3{}, on_entry3)
    MIDDLEWARE_CLASS_END_ITEM(s2, e4, guard3{}, on_entry3)
  )
  )
 private:
  int sml_num_{0};
};

sml::sm<HjSml, sml::thread_safe<std::recursive_mutex>>& GetSmlInstance();  // inline
#define HJ_TRIGGER_EVENT(event) hj_sml::GetSmlInstance().process_event(hj_sml::event{})
}  // namespace hj_sml

#endif
