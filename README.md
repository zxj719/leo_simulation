# leo_simulation

## kick off!
For your smoothly setting out to launch the leo rover robot simulation, Faye, all you need to do is just clone this repository by opening your terminal and typing as follows:
```bash
cd path/to/your/src
```
change the path name with your ros2_workspace/src path, don't paste above directly, plz.
```bash
git clone https://github.com/zxj719/leo_simulation.git
```
Build it, source it, launch it. In case you get confused again, in the workspace directory and copy that
```bash
colcon build --packages-select my_leo
. install/setup.bash
ros2 launch my_leo my_leo.launch.py
```
Then you should see leo in Gazebo GUI. Next we test the keyboard control, open up a new terminal and input
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Follow the instruction in your current terminal and... move it!

You can also visualize the lidar scanning area, hope you enjoy it!

## 2.18 issue
The problem is, we can't make the slam_toolbox publish the /map data. In other words, slam_toolbox fails to provide the map->odom transform.
Now, open another terminal,
```bash
ros2 topic list
```
you'll notice the /map and /map_metadata as well, but if you look into it,
```bash
ros2 topic echo /map
```
/map didn't publish anything, through my debugging experience, I found that
```bash
ros2 topic echo /tf
```
missing the frame id map. I think it is likely the problem from the .urdf file, or you can check the launch file.

Next command might be useful if you want to check the transformation between 2 frames:
```bash
ros2 run tf2_ros tf2_echo base_link[reference_frame] lidar_link[target_frame]
```



@ UoM-Robotics-Team4
