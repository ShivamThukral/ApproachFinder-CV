## Votenet ROS integration
### Installation Instructions:

Please follow the instructions given [here](https://github.com/facebookresearch/votenet#installation) to install relevant votenet packages. For the purpose of this work, we used the commit [2f6d6d3](https://github.com/facebookresearch/votenet/commit/2f6d6d36ff98d96901182e935afe48ccee82d566)

Note: It suggested that you install exact versions with [Anancoda](https://www.anaconda.com/products/individual) for seamless integration. 

You might need to run the following commands to add missing packages:
```asm
conda install -c conda-forge rospkg
conda install -c conda-forge pyyaml
sudo apt-get install ros-melodic-ros-numpy
```

### Steps to run Votenet in simulation
We ship a trained votenet model to detect tables and toilets in indoor environments. It's placed under *'ros_votenet/scripts/votenet_model'*. 

Please run the following commands in a separate terminal:
```asm
cd simulation
source ./devel/setup.bash
roslaunch my_worlds  my_office_env.launch
roslaunch my_robot spawn.launch x:=4 y:=4
roslaunch teleop_twist_joy teleop.launch
roslaunch my_robot rviz.launch
rosrun my_robot time_sync_subscriber_node
```

For detailed description of above-mentioned steps please refer simulation package. Also, remember to enable the joystick controller.

**Run the rosified votenet for table and toilet detections by:** 

```asm
cd ../votenet
source ./devel/setup.bash
rosrun ros_votenet ros_votenet_detection.py 
```


Subscribe to "/votenet/bboxRviz" topic in RVIZ for bounding box visualisations.

### Results:

<img src="../images/votenet/votenet-round-table.png" height="300" width="360"> <img src="../images/votenet/votenet-conference-table.png" height="300" width="360">
