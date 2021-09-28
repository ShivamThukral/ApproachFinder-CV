# ApproachFinder-CV
A real-time computer vision algorithm to find potential docking locations indoor environments.


## Introduction
TODO: Project Brief / Abstract

## Installation Instructions

**Installation Prerequisites**
1. Install [Ubuntu 18.04 64-bit](https://ubuntu.com/)
2. Install [ros-melodic-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu)

This repository is divided into 3 standalone ROS packages: simulation, votenet and docking locations.
- simulation: provides a robot and simulation environments for real-time testing for the vision pipeline.
- votenet: contains ROSified version of votenet trained to detect tables and toilet in indoor environments. 
- docking_locations: consist our computer vision pipeline to find docking locations and generates desirability cost-maps. 

Each of the above-mentioned folders contains a README which summarises exact steps to install module specific packages. Please refer each README file for further installation instructions.

## Demo in Simulation
1. Launch Simulation Environment
2. Launch Votenet
3. Launch CV pipeline


## Results

