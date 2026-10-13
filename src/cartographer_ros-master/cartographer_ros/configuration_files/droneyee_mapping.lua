-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"                   --包含的cartographer里的lua文件
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息

  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "base_link",              -- 将所有传感器数据转换到这个坐标系下，如果有imu的，就写imu的link，如果没有，就写base_link或者footprint，
                                            --因为cartographer会将所有传感器进行坐标变换到tracking_fram坐标系下，每个传感器的频率不一样，imu频率远高于雷达的频率，这样做可以减少计算量
  published_frame = "base_link",            -- tf: map -> footprint 自己bag的tf最上面的坐标系的名字
  
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = true,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上，没啥用
  publish_tracked_pose=true,
  --use_pose_extrapolator = false,            -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿，前端计算的位姿更准

  use_odometry = false,                     -- 是否使用里程计,如果使用要求一定要有odom的tf *
  use_nav_sat = false,                      -- 是否使用gps *topic形式订阅，不可订阅多个里程计/gps/landmark，要注意做重映射
  use_landmarks = false,                    -- 是否使用landmark *
  num_laser_scans = 1,                      -- 是否使用单线激光数据，可订阅多个
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据

  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布子图数据的时间间隔
  pose_publish_period_sec = 75e-3,           -- 位姿发布间隔
  trajectory_publish_period_sec = 75e-3,    -- 轨迹发布间隔

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,             --设成0.1即来10帧用1帧
  fixed_frame_pose_sampling_ratio = 1.,     --某个传感器不准，可以降低其使用频率
  imu_sampling_ratio = 1.,                  --如若不准，一般直接弃用即可
  landmarks_sampling_ratio = 1.,

  -- 处理飞控 IMU 时间戳可能乱序的问题
  -- 当检测到乱序消息时，忽略该消息而不是崩溃
  ignore_out_of_order = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true   --之前进行include已经包含在该文件中了，这里进行重写，列出的都为比较重要的参数
--在这里进行重写就会覆盖子文件中的参数



TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.
--TRAJECTORY_BUILDER_2D.min_z = 0.2
--TRAJECTORY_BUILDER_2D.max_z = 1.4
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30. --点云最大有效距离

--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false    
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 100. --扫描匹配点云和地图匹配程度，值越大，点云和地图匹配置信度越高
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30. --一个子图插入多少个节点
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes = 0. -- 2倍的num_range_data以上（自动优化）
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 0.5

--回环检测阈值，如果不稳定有错误匹配，可以提高这两个值，场景重复可以降低或者关闭回环
POSE_GRAPH.constraint_builder.min_score = 0.9    -- 最小匹配分数
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9  -- 全局定位最小分数



return options

