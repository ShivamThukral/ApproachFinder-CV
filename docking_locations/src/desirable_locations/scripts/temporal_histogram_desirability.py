#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from rviz_tools import RvizMarkers
from sensor_msgs.msg import PointCloud2
from convert_PCD import convertCloudFromRosToOpen3d,convertCloudFromOpen3dToRos
import open3d as o3d

#-------------------------------------------
# initial belief - none of the locations are desirable
temporal_pcd  = o3d.geometry.PointCloud()
NEIGHBOUR_DISTANCE = 0.01  # 1cm
#-------------------------------------------

class temporal_desirability_histogram:
    def __init__(self):
        #subscribe to the pcd - sensor measurement
        self.subscriber_ = rospy.Subscriber("/cost_map",PointCloud2 , self.callback, queue_size=1)
        self.pub_ = rospy.Publisher("/temporal_desirability", PointCloud2, queue_size=1)
        print('Subscriber and Publisher Set...')

    def getClosestNeighbour(self, point):
        closest_point = np.array([-1.0,-1.0,-1.0])  # default is not match found
        for temporal_points in np.asarray(temporal_pcd.points):
            # only consider xy values
            if ((temporal_points[0] - point[0])**2 + (temporal_points[1] - point[1])**2) <= NEIGHBOUR_DISTANCE:
                return temporal_points
        return closest_point


    def callback(self, current_cost_map):
        global temporal_pcd
        current_pcd = convertCloudFromRosToOpen3d(current_cost_map)
        # use the first point cloud to set the bounds
        if temporal_pcd.is_empty():
            #set all the z-values to zero - no desirability and voxelise the pcd for histogram filter
            xyz = np.asarray(current_pcd.points)
            xyz[:,2] = 0.0 # set z-to zero
            temporal_pcd.points = o3d.utility.Vector3dVector(xyz)
            temporal_pcd = temporal_pcd.voxel_down_sample(voxel_size=0.05)
            print("Used the first to set the dimensions..")
        else:
            '''
                3 cases are possible 
                case 1: Point is not present - add this point with a value proportional to its desirability
                case 2: Point is present : (Value might increase,decrese,remain same) -  in all the cases add the value proportional to its sensor value
                case 3: Point is absent : Decrease the value at this index
                Question? Do I need normalisation in this case?If yes, then case 3 is automatically solved.
            '''
            print('Inside Else..')
            print("Temporal pcd points: " , np.asarray(temporal_pcd.points).shape)
            # voxelise the current map - need to decrease the time complexity
            current_pcd = current_pcd.voxel_down_sample(voxel_size=0.05)
            print("Sensed pcd points: " , np.asarray(temporal_pcd.points).shape)
            # traverse the point in the sensed pcd and check for cases
            for point in np.asarray(current_pcd.points):
                closestPoint = self.getClosestNeighbour(point)
            print("yes")

            # nearest neighbour approach to find closest point - I cannot do an equality check due to voxelisation of 2cm in xyz

            # print(np.asarray(current_pcd.points).shape)
            # for point in np.asarray(current_pcd.points):
            #     closestPoint = self.getClosestNeighbour(point)
            #     if closestPoint[0]!=-1 and closestPoint[1] !=-1:
            #         count+=1
            #
            # print(count)
        ros_cloud = convertCloudFromOpen3dToRos(temporal_pcd,"base_footprint")
        self.pub_.publish(ros_cloud)


if __name__ == '__main__':
    rospy.init_node('temporal_histogram')
    temporal = temporal_desirability_histogram()
    rospy.spin()
    print('temporal desirability node shutdown....')