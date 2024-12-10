include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
    map_builder = MAP_BUILDER,  
    trajectory_builder = TRAJECTORY_BUILDER,  
    map_frame = "map",  -- map frame name 
    tracking_frame = "base_link",  -- tracking frame name
    published_frame = "base_link",  -- published frame name 
    odom_frame = "odom",  -- name of the odometer frame
    provide_odom_frame = false,  -- whether to provide the odometer frame
    publish_frame_projected_to_2d = true,  -- whether to publish 2d gesture  
    use_odometry = false,  -- 
    use_nav_sat = false,  --  
    use_landmarks = false,  -- 
    num_laser_scans = 1,  -- 
    num_multi_echo_laser_scans = 0,  
    num_subdivisions_per_laser_scan = 1,  -- number of subdivisions for each laser scan
    num_point_clouds = 0,  -- number of cloud points
    lookup_transform_timeout_sec = 0.2, --0.2  -- timeout for finding transformations (seconds)  
    submap_publish_period_sec = 1, --0.001  -- submap release cycle (seconds)
    pose_publish_period_sec = 0.01, --0.01  -- attitude release period (seconds)
    trajectory_publish_period_sec = 2,  
    rangefinder_sampling_ratio = 1.0,  
    odometry_sampling_ratio = 1.,  
    fixed_frame_pose_sampling_ratio = 1.,  -- fixed frame attitude sampling ratio  
    imu_sampling_ratio = 1.,  
    landmarks_sampling_ratio = 1.,  
    -- publish_tracked_pose = on
  }
  
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 8

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10
TRAJECTORY_BUILDER_2D.min_range = 0.12  -- ignore anything smaller than the robot radius, limiting it to the minimum scan range of the lidar
TRAJECTORY_BUILDER_2D.max_range = 5.0  -- the maximum scanning range of the lidar
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0 -- Restricted to maximum LiDAR scanning range
--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1
TRAJECTORY_BUILDER_2D.use_imu_data = false    -- whether use IMU data
-- TRAJECTORY_BUILDER_2D.min_z = -0.1
-- TRAJECTORY_BUILDER_2D.max_z = 0.1
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45 --default 0.49

--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 1.0  -- Modify 1.0 to 0.1, increased sensitivity to movement
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1
POSE_GRAPH.constraint_builder.min_score = 0.57  -- Modify 0.55 to 0.65, the minium score of Fast csm, can be optimized above this score 
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55  -- Modify 0.6 as 0.7, Minimum global positioning score below which global positioning is considered currently inaccurate

POSE_GRAPH.optimize_every_n_nodes = 2 --fits every n submaps with the orignal map

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER.pure_localization = true
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = 0.523599
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
  
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 45.0  -- Increase for tighter match to the map
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0 -- Higher values penalize rotations
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15.0 -- Higher values penalize translations
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 --default 10 -- Increase for better optimization
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150

--TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1 -- Default is 0.05; larger values reduce detail and computational cost
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1 --Defult 0.3 map building constraints's ration
-- POSE_GRAPH.global_sampling_ratio = 0.001 --default 0.003

POSE_GRAPH.constraint_builder.max_constraint_distance = 2
POSE_GRAPH.global_constraint_search_after_n_seconds = 20.0 
return options

