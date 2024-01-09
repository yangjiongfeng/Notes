# 2-cartographer参数



### cartographer_ros参数
路径：carto_ws/src/cartographer_ros/cartographer_ros/configuration_files/backpack_2d.lua


include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- 用来发布子图的ROS坐标系，通常为map
  map_frame = "map",
  -- SLAM跟随的坐标系ID
  -- 将所有传感器数据转换到这个坐标系下，如果有imu的，就写imu的link，如果没有，就写base_link或者footprint，
  --因为cartographer会将所有传感器进行坐标变换到tracking_fram坐标系下，每个传感器的频率不一样，imu频率远高于雷达的频率，这样做可以减少计算量
  tracking_frame = "base_link",
  -- 发布map到published_frame之间的tf
  published_frame = "odom",
  -- 位于map_frame和published_frame之间，用来发布本地SLAM结果(非闭环)，通常为odom
  odom_frame = "odom",
  -- 是否提供里程计
  -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint，如果为false tf树为map->footprint
  provide_odom_frame = false,
  -- 只发布二维位姿态(不包含俯仰角)是否将坐标系投影到平面上，没啥用。
  publish_frame_projected_to_2d = false,
  -- 是否使用里程计数据
  use_odometry = true,
  -- 是否使用GPS定位
  use_nav_sat = false,
  -- 是否使用路标
  use_landmarks = false,
  -- 订阅laser scan topic的个数
  num_laser_scans = 1,
  -- 订阅多回波技术的laser scan topic的个数
  num_multi_echo_laser_scans = 0,
  -- 分割雷达数据的个数，1帧数据被分成几次处理，一般为1
  num_subdivisions_per_laser_scan = 1,
  -- 订阅点云topics的个数
  num_point_clouds = 0,
  --使用tf查找变换的超时秒数
  lookup_transform_timeout_sec = 0.2,
  -- 发布submap的周期间隔
  submap_publish_period_sec = 0.3,
  -- 发布姿态的的周期间隔
  pose_publish_period_sec = 5e-3,
  -- 轨迹发布周期间隔
  trajectory_publish_period_sec = 30e-3,
  -- 测距仪的采样率
  rangefinder_sampling_ratio = 1.,
  -- 里程计数据采样率
  odometry_sampling_ratio = 1.,
  -- 固定的frame位姿采样率
  fixed_frame_pose_sampling_ratio = 1.,
  -- imu数据采样率
  imu_sampling_ratio = 1.,
  -- 路标采样率
  landmarks_sampling_ratio = 1.,
}

--使用2D轨迹
MAP_BUILDER.use_trajectory_builder_2d = true

--雷达数据的最近值
TRAJECTORY_BUILDER_2D.min_range = 0.1
--雷达数据的最远值
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.
--使用imu数据
TRAJECTORY_BUILDER_2D.use_imu_data = false
--是否使用相关性暴力匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2    --0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)   --math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 10   --5.

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.) -- math.rad(20.)

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100.  --10.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.   --40.

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 12.0
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 12.0
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5 --1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5 --1e5
--POSE_GRAPH.optimize_every_n_nodes = 60   --set to 0 disable global slam
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 2000   --set to 0 disable global slam
return options


### 前端参数
路径：carto_ws/src/cartographer_ros/cartographer_ros/configuration_files/trajectory_2d.lua


TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,                 -- 是否使用imu数据
  min_range = 0.,                      -- 雷达数据最小值
  max_range = 30.,                     -- 雷达数据最大值
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,        -- 超过最大距离范围的数据点用这个距离代替
  num_accumulated_range_data = 1,      -- 几帧有效的点云数据进行一次扫描匹配
  voxel_filter_size = 0.025,           -- 体素滤波的立方体的边长

  -- 体素滤波器用于生成稀疏点云以进行扫描匹配
  adaptive_voxel_filter = {
    max_length = 0.5,                  -- 尝试确定最佳的立方体边长, 边长最大为0.5
    min_num_points = 200,              -- 如果存在 较多点 并且大于'min_num_points', 则减小体素长度以尝试获得该最小点数
    max_range = 50.,                   -- 距远离原点超过max_range 的点被移除
  },

  -- 闭环检测的自适应体素滤波器, 用于生成稀疏点云以进行闭环检测
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },


  -- 是否使用 real_time_correlative_scan_matcher 为ceres提供先验信息
  -- 计算复杂度高 , 但是很鲁棒 , 在odom或者imu不准时依然能达到很好的效果
  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,               -- 线性搜索窗口的大小
    angular_search_window = math.rad(20.),    -- 角度搜索窗口的大小
    translation_delta_cost_weight = 1e-1,     -- 用于计算各部分score的权重
    rotation_delta_cost_weight = 1e-1,
  },

  -- ceres匹配的一些配置参数
  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  -- 为了防止子图里插入太多数据, 在插入子图之前之前对数据进行过滤
  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  imu_gravity_time_constant = 10.,

  -- 子图相关的一些配置
  submaps = {
    num_range_data = 90,                 -- 一个子图里插入雷达数据的个数的一半
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",    -- 地图的种类, 还可以是tsdf格式
      resolution = 0.05,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
    },
  },
}


  -- 是否使用IMU数据
  use_imu_data = true,
  -- 深度数据最小范围
  min_range = 0.1,
  -- 深度数据最大范围
  max_range = 15.,
  -- 传感器数据超过有效范围最大值时，按此值来处理
  missing_data_ray_length = 5.,
  -- 是否使用实时回环检测来进行前端扫描匹配
  use_online_correlative_scan_matching = true,
  -- 运动过滤，检测运动变化，避免机器人静止时插入数据
  motion_filter.max_angle_radians


### 后端参数
路径：carto_ws/src/cartographer_ros/cartographer_ros/configuration_files/pose_graph.lua

POSE_GRAPH = {
  optimize_every_n_nodes = 90,               -- 每隔多少个节点执行一次后端优化
  -- 约束构建的相关参数
  constraint_builder = {
    sampling_ratio = 0.3,                    -- 对局部子图回环检测的计算频率, 数值越大计算次数越多
    max_constraint_distance = 15.,           -- 对局部子图进行回环检测时能成为约束的最大距离
    min_score = 0.55,                        -- 对局部子图进行回环检测时的最低分数阈值
    global_localization_min_score = 0.6,     -- 对整体子图进行回环检测时的最低分数阈值
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,                      -- 打印约束计算的log

    -- 基于分支定界算法的2d粗匹配器
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },

    -- 基于ceres的2d精匹配器
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },

    -- 基于分支定界算法的3d粗匹配器
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },

    -- 基于ceres的3d精匹配器
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,

  -- 优化残差方程的相关参数
  optimization_problem = {
    huber_scale = 1e1,                         
    acceleration_weight = 1e3,                  -- imu线加速度的权重
    rotation_weight = 3e5,                      -- imu的旋转的权重
    local_slam_pose_translation_weight = 1e5,   -- 前端结果残差的权重
    local_slam_pose_rotation_weight = 1e5,      
    odometry_translation_weight = 1e5,           -- 里程计残差的权重
    odometry_rotation_weight = 1e5,
    fixed_frame_pose_translation_weight = 1e1,   -- gps残差的权重
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,                   -- 在建图结束后执行一次全局优化, 不要求实时性, 迭代次数多
  global_sampling_ratio = 0.003,                    -- 纯定位时候查找回环的频率
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,   -- 纯定位时多少秒执行一次全子图的约束计算
}



  -- Fast csm最低分数，高于此分数才进行优化
  sampling_ratio.min_score = 0.65
  -- 全局定位最小分数，低于此分数则认为目前全局定位不准确
  sampling_ratio.global_localization_min_score = 0.7





