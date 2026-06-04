include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",         -- Frame associated with IMU data (to align scans with gravity vector)
  published_frame = "odom",            -- Cartographer publishes transform map -> published_frame
  odom_frame = "odom",                 -- Used as odom_frame name
  provide_odom_frame = false,          -- Set false because external EKF publishes odom -> base_footprint
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,                 -- Subscribe to /odom (remapped to /odometry/filtered)
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
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ── 2D Trajectory Builder Settings ───────────────────────────────────────
TRAJECTORY_BUILDER_2D.use_imu_data = true           -- Use BNO085 pitch/roll for gravity projection
TRAJECTORY_BUILDER_2D.min_range = 0.15              -- RPLIDAR A1M8 minimum range
TRAJECTORY_BUILDER_2D.max_range = 6.0               -- RPLIDAR A1M8 reliable range limit
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- Correlative scan matching is disabled in straight, symmetric corridors to
-- prevent false-positive global/local alignments that shift the robot along the hallway.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

-- Use smaller submaps so that they complete faster and get aligned globally.
-- This reduces accumulated local drift in narrow-FOV setups.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50

-- ── Ceres Scan Matcher Tuning ─────────────────────────────────────────────
-- Since bumps cause LiDAR scan plane tilt, we rely more on the EKF prior.
-- Increasing translation/rotation weights relative to occupied_space_weight
-- forces the scan matcher to trust wheel odometry and IMU heading over scan alignments.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0   -- Heavy penalty for translating away from EKF
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 70.0      -- Heavy penalty for rotating away from IMU/EKF heading

-- ── Pose Graph Optimization (Global Loop Closures) ─────────────────────────
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

return options
