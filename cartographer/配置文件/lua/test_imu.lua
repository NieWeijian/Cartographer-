-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--TRAJECTORY_BUILDER.pure_localization = true
--POSE_GRAPH.optimize_every_n_nodes = 20

-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50                                                  -- 每个submap的scan总数为num_range_data 的2倍，前一半进行初始化而不进行匹配，后一半进行匹配
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 2
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true 
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 9.803
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true                                 -- 是否使用 real_time_correlative_scan_matcher 解决 在线的扫描匹配 问题，从而为ceres提供先验信息
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
POSE_GRAPH.optimization_problem.huber_scale = 1e2 
 
POSE_GRAPH.constraint_builder.min_score = 0.65                                                    -- 扫描匹配分数的阈值，低于该阈值时不考虑匹配。 低分表示扫描和地图看起来不相似
 
--optimization 
 
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight=1e5                            -- 基于local SLAM的姿势在连续节点之间进行平移的权重
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight=1e5                               -- 基于里程计的姿势在连续节点之间进行平移的权重
POSE_GRAPH.optimization_problem.odometry_translation_weight=1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight=1e5           
POSE_GRAPH.optimization_problem.acceleration_weight = 1e1                                         -- IMU加速度的权重
POSE_GRAPH.optimization_problem.rotation_weight = 3e3                                             -- IMU旋转项的权重
POSE_GRAPH.max_num_final_iterations = 150                                                         -- 在建图结束之后会运行一个新的全局优化，不要求实时性，迭代次数多
POSE_GRAPH.optimize_every_n_nodes = 50                                                            -- 每几个节点（节点是什么？scan?）执行一次优化，设为0即关闭后端优化
                      
 
 
return options 
 
