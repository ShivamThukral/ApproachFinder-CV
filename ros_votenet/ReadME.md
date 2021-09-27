## Votenet ROS integration
Activate the conda environment
```
conda activate votenet
```
Source NVCC 10 for votenet
```
source ~/cuda/source_cuda10
```
Run the node to do detections
```
rosrun ros_votenet ros_votenet_detection.py 
```

We can do a rostopic echo to find whats published from this node
```
rostopic echo /votenet/detections
```
Subscribe to "/votenet/bboxRviz" topic in RVIZ for bounding box visualisations.