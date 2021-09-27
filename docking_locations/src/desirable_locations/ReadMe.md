## Desirable Locations

Run the code to find locations around the table:
```bash
rosrun desirable_locations find_locations_node 
```

Activate the open3d_env and run the code to find the cost map for these desirable locations
```bash
rosrun desirable_locations cost_map.py
```

Rviz : 
- Subscribe to "/pcl_processing/locations_rviz" to see the locations as small spheres
- Subcribe to "/cost_map" for the cost map