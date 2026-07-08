# SC-PGO ROS 2 Runtime Package

This is the Scan Context pose graph optimization back-end used by the parent [`FAST_LIO_SLAM_ROS2`](https://github.com/ravnbudde/FAST_LIO_SLAM_ROS2) stack.

For full SLAM usage, composed launch commands, topic wiring, Fast-LIO integration, and bag verification, see the parent repository README.

## What This Fork Provides

- ROS 2 package name: `aloam_velodyne`
- Standalone executable: `alaserPGO`
- Composable node plugin: `aloam_velodyne::LaserPGONode`
- Scan Context loop detection
- ICP loop constraint calculation
- GTSAM/iSAM2 pose graph optimization
- Optional GNSS altitude factor from `sensor_msgs/msg/NavSatFix`
- RViz config and offline Open3D map utility

## Why This Package Is Trimmed

The original SC-A-LOAM repository includes the A-LOAM front-end, KITTI helper, Docker files, sample maps, images, and other examples. In this stack, Fast-LIO is the odometry front-end, so this package only keeps the SC-PGO back-end and the small utilities needed to inspect its output.

Removed from this fork/runtime branch:

- A-LOAM scan registration, odometry, and mapping nodes
- KITTI helper
- Docker files
- sample datasets and result images
- stale Python bytecode/cache files

Kept intentionally:

- `src/laserPosegraphOptimization.cpp`
- Scan Context headers and implementation
- `rviz_cfg/aloam_velodyne.rviz`
- `utils/python/makeMergedMap.py` and color tables for offline map inspection

## Inputs

SC-PGO subscribes to these internal topic names:

```text
/aft_mapped_to_init                 nav_msgs/msg/Odometry
/velodyne_cloud_registered_local    sensor_msgs/msg/PointCloud2
/gps/fix                            sensor_msgs/msg/NavSatFix, optional
```

Use launch remaps to connect them to your odometry front-end topics.

## Outputs

```text
/aft_pgo_odom        nav_msgs/msg/Odometry
/aft_pgo_path        nav_msgs/msg/Path
/aft_pgo_map         sensor_msgs/msg/PointCloud2
/loop_scan_local     sensor_msgs/msg/PointCloud2
/loop_submap_local   sensor_msgs/msg/PointCloud2
```

Saved files under `save_directory`:

```text
optimized_poses.txt
odom_poses.txt
times.txt
singlesession_posegraph.g2o
Scans/*.pcd
SCDs/*.scd
```

`Scans/` and `SCDs/` are recreated when SC-PGO starts. Use a new `save_directory` for each run you want to keep.

## Standalone Usage

```bash
colcon build --packages-select aloam_velodyne
source install/setup.bash

ros2 run aloam_velodyne alaserPGO --ros-args \
  -p save_directory:=/tmp/sc_pgo/ \
  -r /aft_mapped_to_init:=/fast_lio_slam/odometry/local \
  -r /velodyne_cloud_registered_local:=/fast_lio_slam/points/body \
  -r /gps/fix:=/gps/fix
```

The parent full-stack launch uses the composed plugin instead of the standalone process.
