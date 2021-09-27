## Simulation Environment in Gazebo

Launch table chair world in the gazebo environment:
```bash 
roslaunch my_worlds my_table_chair.launch
```
Spawn the robot in the gazebo environment:
```bash 
roslaunch my_robot spawn.launch
```
Launch the robot model in RVIZ:
```bash
roslaunch my_robot rviz.launch
```
Rviz Settings:
 - Global Fixed Frame : "base_footprint"
 - Add Robot Model 
 - Add TF
 - Add PointCloud and image from published topic list
 
 To control the movement of the robot:
```bash
 rosrun my_robot teleop_twist_keyboard.py
```
To convert the pointcloud from "camera_optical_depth_frame" to "base_footprint" run the node:
```bash
rosrun my_robot kinect_subscriber_node
``` 
