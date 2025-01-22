## 清洁日志消息
float32 clean_speed             # 清洁速度 单位：m/s
float32 surface_clean_area      # 清洁水面面积 单位：m2
float32 bottom_clean_area       # 清洁池底面积
float32 wall_clean_area         # 清洁池壁面积
float32 water_line_clean_area   # 清洁水线面积
float32 pool_area               # 泳池面积
float32 pool_volume             # 泳池体积
float32 pool_depth              # 泳池最大深度
float32 pool_little_depth       # 泳池最小深度
int32   pool_shape              # 泳池形状
string  map_line_file_path      # 地图轨迹文件路径
int32   avoidObstaclesCount     # 避障次数
int32   getOutCount             # 脱困次数
int32   multiple_bottom         # 多层池底 1-单层 2-多层

#### 以下用于埋点的消息
## 清洁事件消息- 与清洁日志消息高度重叠

## 清洁不同区域共同信息
float32 clean_area             # 清扫面积
int32 trapped_count            # 被困次数

## 水面清洁信息
int32 edge_time                # 延边时长

## 池底清洁信息 - 无多余信息

## 池壁清洁信息 - 无多余信息

## 水线清洁信息 - 无多余信息

## 多平台清洁信息
int32 fall_count               # 跌落次数
int32 avoid_fall               # 防跌落次数
int32[] clean_time               # 清洁时长
float32[] deepth_list          # 深度列表
