# ApproachFinder-CV

A real-time computer vision algorithm to find potential docking locations indoor environments.

## Introduction

A smart wheelchair improves the quality of life for older adults by supporting their mobility independence. Some
critical maneuvering tasks, like table docking and doorway passage, can be challenging for older adults in wheelchairs,
especially those with additional impairment of cognition, perception or fine motor skills. Supporting such functions in
a shared manner with robot control seems to be an ideal solution. Considering this, we propose to augment smart
wheelchair perception with the capability to identify potential docking locations in indoor scenes.

[ApproachFinder-CV](https://github.com/ShivamThukral/ApproachFinder-CV) is a computer vision pipeline that detects safe docking poses and estimates their desirability weight based on
hand-selected geometric relationships and visibility. Although robust, this pipeline is computationally intensive. We
leverage this vision pipeline to generate ground truth labels used to train an end-to-end differentiable neural net that
is 15x faster. 

[ApproachFinder-NN](https://github.com/ShivamThukral/ApproachFinder-NN) is a point-based method that draws motivation from Hough voting and uses deep point
cloud features to vote for potential docking locations. Both approaches rely on just geometric information, making them
invariant to image distortions. A large-scale indoor object detection dataset, SUN RGB-D, is used to design, train and
evaluate the two pipelines.

Potential docking locations are encoded as a 3D temporal desirability cost map that can be integrated into any real-time
path planner. As a proof of concept, we use a model predictive controller that consumes this 3D costmap with efficiently
designed task-driven cost functions to share human intent. This [wheelchair navigation](https://github.com/ShivamThukral/Wheelchair-Navigation) controller outputs a nominal path that is safe,
goal-oriented and jerk-free for wheelchair navigation.

## Installation Instructions

**Installation Prerequisites**

1. Install [Ubuntu 18.04 64-bit](https://ubuntu.com/)
2. Install [ros-melodic-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu)

This repository is divided into 3 standalone ROS packages:

- **simulation**: provides a robot and simulation environments for real-time testing for the vision pipeline.
- **votenet**: contains ROSified version of votenet trained to detect tables and toilet in indoor environments.
- **docking_locations**: consist our computer vision pipeline to find docking locations and generates desirability
  cost-maps.

Each of the above-mentioned folders contains a README which summarises exact steps to install module specific packages.
Please refer each README file for further installation instructions and demo instructions.

## Demo in Simulation

You can run the demo by following these 3 steps:

1. Simulation Environment
    1. Launch the simulation environment in Gazebo:
       ```asm
        roslaunch my_worlds my_office_env.launch
       ```
    2. Spawn the robot at specified location:
       ```asm
         roslaunch my_robot spawn.launch x:=4 y:=4
       ```
    3. Publish pointcloud to votenet:
       ```asm
         rosrun my_robot time_sync_subscriber_node
       ```
    4. Launch RVIZ to visualise the results
       ```asm
         roslaunch my_robot rviz.launch rvizconfig:=demo.rviz
       ```
    5. Joytick Controller to drive the robot
       ```asm
         roslaunch teleop_twist_joy teleop.launch
       ```
2. Launch Votenet
    1. Launch votenet to detect tables and toilets:
       ```asm
         rosrun ros_votenet ros_votenet_detection.py 
       ```
3. Launch CV pipeline
    1. Launch docking locations node:
       ```asm
         rosrun desirable_locations find_locations_approx_node 
       ```
    2. Launch costmap node:
       ```asm
         rosrun desirable_locations cost_map_heading.py
       ```

For further details please refer each sub-directory.

## Simulation Results:

<img src="images/results/teaser.png">

## How to run ApproachFinder-CV on your data?

1. Publish the point cloud on topic *"/camera/depth/points"*. This should be in up-right position (z-axis up).
2. Publish depth image on topic *"/camera/depth/image_raw"*. This channel expects the depth values in meters.
3. Publish the camera information on topic *"/camera/depth/camera_info"*. Please refer here to know more
   about [CameraInfo](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) message.
4. Follow steps 2 and 3 from "Demo in Simulation".

### Results on [SUN RGB-D](https://rgbd.cs.princeton.edu/) dataset:

<img src="images/results/Results.png" >
