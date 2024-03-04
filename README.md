# 2024.3.2 version
## 1. imu, lidar, tf work well.
## 2. camera cancelled.
## 3. slam_toolbox can publish the /map and bonded with odom frame correctly.
# 2024.3.3 v3
## 1. added remote control(keyboard in xterm, joystick in /dev/input/js0)
## 2. integreted control with slam and map_maker in one launch file.
## 3. added camera plugin
## 4. visualize in rviz but invisible for robot body.

after launch my_leo.launch.py and map_maker.launch.py, new a terminal
```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "map_url: './src/my_leo/maps/sim_map'"
```
