// @file hj_utils.h
// @brief
//
// Copyright 2024 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-12-10)
#ifndef SRC_BASE_FRAMEWORK_INCLUDE_UTILS_H_
#define SRC_BASE_FRAMEWORK_INCLUDE_UTILS_H_

#include <string>
#define RECORD_TIMESTAMP hj_bf::getTimeNowStr(__FILE__, __LINE__)
namespace hj_bf {
std::string getTimeNowStr(const std::string& file, int line);

}  // namespace hj_bf

#endif  // SRC_BASE_FRAMEWORK_INCLUDE_UTILS_H_
