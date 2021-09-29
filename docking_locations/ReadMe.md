## Docking Locations and Temporal Desirability

### Installation Instructions:
- Install [PCL 1.8](https://pointclouds.org/downloads/)
- Install [Eigen3](https://ubuntu.pkgs.org/18.04/ubuntu-universe-amd64/libeigen3-dev_3.3.4-4_all.deb.html) 
- Install [OpenCV](https://docs.opencv.org/3.4.15/d7/d9f/tutorial_linux_install.html)
- Install [Open3D](http://www.open3d.org/docs/release/getting_started.html)

### Steps to find docking locations in simulation:

**Run the ros-node to find docking locations aound indoor objects in a scene:**
```asm
rosrun desirable_locations find_locations_approx_node 
```

### Temporal Desirability: 
Temporal desiability is a 3D costmap which has X,Y and heading. We can visualise XY grid maps corresponding to each binned heading. 
For this, first launch the cost_map node and then configure the RVIZ configurations.

**Run the code to find the cost map for the docking locations by:**
```asm
rosrun desirable_locations cost_map_heading.py
```

**Finally, open the configuration file under *'simulation/src/my_robot/config'* with rviz through GUI or by running this command:**
```asm
roslaunch my_robot rviz.launch rvizconfig:=my_robot_new.rviz
```

## Results:

<img src="../images/docking_locations/docking_locations_conference.png" height="330" width="400"> <img src="../images/docking_locations/docking_locations_round.png" height="330" width="400">
<img src="../images/docking_locations/desirability_table.png" height="330" width="400"> <img src="../images/docking_locations/desirability_circular.png" height="330" width="400">
