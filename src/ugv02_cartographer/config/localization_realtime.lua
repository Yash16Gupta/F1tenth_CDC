include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",  -- Map frame name
    tracking_frame = "imu",  -- The frame to track
    published_frame = "base_link",  -- Frame to publish poses
    odom_frame = "odom",  -- Name of the odometer frame
    provide_odom_frame = false,  -- Do not publish odometry frame
    publish_frame_projected_to_2d = true,  -- Publish poses in 2D
    use_odometry = false,  -- Disable odometry usage
    use_nav_sat = false,  -- Disable GPS usage
    use_landmarks = false,  -- Disable landmarks
    num_laser_scans = 1,  -- Single laser scan input
    num_multi_echo_laser_scans = 0,
    num_point_clouds = 0,  -- No point clouds
    num_subdivisions_per_laser_scan = 1,  -- No scan subdivision
    lookup_transform_timeout_sec = 0.2,  -- Transform lookup timeout
    submap_publish_period_sec = 0.5,  -- Submap publishing period
    pose_publish_period_sec = 0.01,  -- Pose publishing period
    trajectory_publish_period_sec = 0.5,  -- Trajectory publishing period
    rangefinder_sampling_ratio = 0.8,  -- LiDAR data sampling ratio
    fixed_frame_pose_sampling_ratio = 1.0,  -- Fixed frame sampling ratio
    imu_sampling_ratio = 1.0,  -- IMU sampling (not used)
    landmarks_sampling_ratio = 1.0,  -- Landmarks sampling (not used)
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 7

TRAJECTORY_BUILDER.pure_localization = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.001

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50.0

POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.matcher_translation_weight = 1e3
POSE_GRAPH.matcher_rotation_weight = 1e3
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.max_num_final_iterations = 200
-- POSE_GRAPH.num_background_threads = 7

return options

