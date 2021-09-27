## Simulation Environment in Gazebo
Please download the gazebo models from [here](https://data.nvision2.eecs.yorku.ca/3DGEMS/) and save it under *my_worlds/models* directory.
Compile the code using catkin_make command and source the files.
### Steps to launch the simluation environment:

**Launch any world environment from 'my_worlds/launch' folder by:**
```bash 
roslaunch my_worlds <file_name>.launch
eg. roslaunch my_worlds  office_env_large.launch
```
**Spawn the robot in the gazebo environment by supplying XY locations:**
```bash 
roslaunch my_robot spawn.launch x:=5 y:=5
```
**Launch the robot model in RVIZ with specified RVIZ configuration file:**
```bash
roslaunch my_robot rviz.launch rvizconfig:=simple_viz.rviz
```

If everything works fine then you should see robot model, point cloud and images of the scene.

**Robot Control: keyboard or joystick**

```bash
 rosrun my_robot teleop_twist_keyboard.py
 OR
 roslaunch teleop_twist_joy teleop.launch
```

##Results:

<img src="../images/simulation/robot-gazebo.jpg" width="300" height="300">
<img src="../images/simulation/robot-rviz.png" width="300" height="300">