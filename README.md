# SC-PGO ROS 2

This branch is a trimmed ROS 2 package that keeps only the Scan Context pose graph optimization node from SC-A-LOAM. It is intended to run behind an external odometry front-end such as Fast-LIO.

## What is included

- `alaserPGO`: Scan Context loop detection, ICP loop constraints, GPS altitude factor, and GTSAM/iSAM2 pose graph optimization.
- `include/scancontext`: Scan Context descriptor and loop-candidate search.
- `rviz_cfg/aloam_velodyne.rviz`: RViz visualization config.
- `utils/python/makeMergedMap.py`: offline Open3D map builder for saved keyframe scans and optimized poses.
- `launch/sc_pgo_fast_lio.launch.py`: launch file with remaps for Fast-LIO odometry, registered cloud, and optional GNSS.

## Required inputs

`alaserPGO` subscribes to these internal topic names:

- `/aft_mapped_to_init` (`nav_msgs/msg/Odometry`)
- `/velodyne_cloud_registered_local` (`sensor_msgs/msg/PointCloud2`)
- `/gps/fix` (`sensor_msgs/msg/NavSatFix`, optional altitude stabilization)

Use launch remaps to connect those to your front-end topics.

## Build

```bash
cd /home/lonewolf/temp_ws
colcon build --packages-select aloam_velodyne
source install/setup.bash
```

## Run With Fast-LIO Bag Topics

```bash
ros2 launch aloam_velodyne sc_pgo_fast_lio.launch.py \
  save_directory:=/home/lonewolf/temp_ws/sc_pgo_out/ \
  use_sim_time:=true \
  odom_topic:=/lonewolf/odometry/local \
  cloud_topic:=/lonewolf/fast_lio/cloud_registered_body \
  gps_topic:=/lonewolf/vectornav/gnss
```

Then play the bag in another terminal:

```bash
ros2 bag play /home/lonewolf/temp_ws/fast_lio_bag
```

`save_directory` is destructive for `Scans/` and `SCDs/`: the node recreates those subdirectories on startup. Use a fresh output directory for each run you want to keep.

## Outputs

Published topics:

- `/aft_pgo_odom`
- `/aft_pgo_path`
- `/aft_pgo_map`
- `/loop_scan_local`
- `/loop_submap_local`

Saved files:

- `optimized_poses.txt`
- `odom_poses.txt`
- `times.txt`
- `Scans/*.pcd`
- `SCDs/*`

## Offline Map Inspection

Edit `utils/python/makeMergedMap.py` so `data_dir` points at the SC-PGO output directory, then run it from the utility directory:

```bash
cd /home/lonewolf/temp_ws/src/SC-A-LOAM-ROS2/utils/python
python3 makeMergedMap.py
```

The script stacks saved keyframe scans using `optimized_poses.txt`, opens an Open3D viewer, and saves a merged map PCD in the output directory.
