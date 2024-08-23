#ifndef __SHM_DATA_H__
#define __SHM_DATA_H__
namespace shm_data {

constexpr const char* MAP_DATA_SHM  = "map_data_shm";
constexpr const char* CLEAN_TRACE_DATA_SHM ="clean_trace_data_shm";
constexpr int32_t  MAX_POINT = 10000+10;

struct MapData { // 地图数据
  float x;
  float y;
  float z;
};

struct CleanTraceData { // 清扫轨迹数据
  float x;
  float y;
  float z;
};

} // namespace shm_data
#endif