// @file hj_utils.cpp
// @brief
//
// Copyright 2024 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2024-12-10)
#include "hj_utils.h"
#include <ctime>
#include <cstring>
#include <sys/time.h>
#include <stddef.h>
#define FILENAME(x) strrchr(x, '/')?strrchr(x,'/')+1:x

namespace hj_bf {
//[2024-12-09 07:26:39,267] [INFO] (node_factory.cpp:284)
std::string getTimeNowStr(const std::string& file, int line) {
  std::string out;
  static char buf[256];
  static int size = sizeof(buf);
  memset(buf, '\0', size);
  time_t now = time(NULL);
  struct tm* local_time = localtime(&now);
  struct timeval tv;
  gettimeofday(&tv,NULL);
  snprintf(buf, size, "[%04d-%02d-%02d %02d:%02d:%02d,%03d] (%s:%d) ",
      local_time->tm_year+1900,
      local_time->tm_mon+1,
      local_time->tm_mday,
      local_time->tm_hour,
      local_time->tm_min,
      local_time->tm_sec,
      static_cast<int>(tv.tv_usec/1000),
      FILENAME(file.c_str()),
      line);
  return buf;
}

}  // namespace hj_bf
