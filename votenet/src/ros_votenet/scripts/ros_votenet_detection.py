#!/usr/bin/env python
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
from ros_votenet.msg import votenetDetection, detectionArray, kinect_data
from geometry_msgs.msg import Point, Pose, Quaternion
from rviz_tools import RvizMarkers
import sys
# insert at 1, 0 is the script path (or '' in REPL)
#sys.path.insert(1, '/home/vcr/UBC/Research/votenet')
from votenet_ros_version import run_votenet

des_frame = "odom"                  # -- enable for simulation
#des_frame = "world"
# SJC dataset
#des_frame = "camera_link"          # -- enable for xtion ASUS
#des_frame = "zed2_camera_center"
#des_frame = "map"                  # ZED CAMERA
OBJECTNESS_THRESHOLD = 0.75         # threshold of object confidence
RVIZ_VIZ_DURATION = 4.0             # in seconds

WHITE_LIST = ['table','toilet']

class Votenet_ROS:
    def __init__(self):
        #self.subscriber_rviz = rospy.Subscriber("/votenet/pcd",PointCloud2 , self.callback, queue_size=1)
        self.subscriber_ = rospy.Subscriber("/camera/data",kinect_data , self.callback_votenet, queue_size=1)
        self.publisher_ = rospy.Publisher("/votenet/detections", detectionArray, queue_size=1)
        self.pcd_array = np.empty([],dtype='float')
        self.input_pcd = PointCloud2()
        self.depth_image = Image()
        self.markers = RvizMarkers(des_frame, '/votenet/bboxRviz')
        print('Subscriber and Publisher Set')

    def callback_votenet(self,data):
        input_cloud = data.scene
        assert isinstance(input_cloud, PointCloud2)  # check for point cloud instance
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(input_cloud)
        print('PCD Recieved of Size :', xyz_array.shape)
        self.pcd_array = xyz_array
        self.input_pcd = input_cloud
        self.depth_image = data.depth_image
        self.find_bouding_boxes()

    def callback(self, input_cloud):
        assert isinstance(input_cloud, PointCloud2)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(input_cloud)
        print('PCD Recieved of Size :', xyz_array.shape)
        self.pcd_array = xyz_array
        self.input_pcd = input_cloud
        self.find_bouding_boxes()
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(xyz_array)
        # o3d.visualization.draw_geometries([pcd])

    #https://github.com/DavidB-CMU/rviz_tools_py/blob/master/example/demo.py
    def publishBoundingBox(self, all_detections):
        # visualise only confident values (>=0.85)
        corners = []
        for i in range(len(all_detections)):
            if all_detections[i][1] < OBJECTNESS_THRESHOLD or (all_detections[i][0] not in WHITE_LIST):
                continue
            for j in range(len(all_detections[i][2])):
                p = Point()
                p.x, p.y,p.z = all_detections[i][2][j][0],all_detections[i][2][j][1],all_detections[i][2][j][2]
                corners.append(p)
        if len(corners) == 0:
            return
        # plot the lines
        width = 0.04
        for i in range(0,len(corners),4):
            path = []
            path.append(corners[i])
            path.append(corners[i+1])
            path.append(corners[i+2])
            path.append(corners[i+3])
            path.append(corners[i])   # to complete the rectangle
            self.markers.publishPath(path, 'orange', width, RVIZ_VIZ_DURATION) # path, color, width, lifetime
        # add the 4 remaining lines in z-axis manually
        index = [1,5,6,2,3,7,8,4]
        for i in range(0,len(corners),8):
            path = []
            for ind in index:
                path.append(corners[i+ind-1])
            self.markers.publishPath(path, 'orange', width, RVIZ_VIZ_DURATION) # point1, point2, color, width, lifetime
        diameter = 0.05
        self.markers.publishSpheres(corners, 'yellow', diameter, RVIZ_VIZ_DURATION) # path, color, diameter, lifetime

    def find_bouding_boxes(self):
        all_detections = run_votenet(self.pcd_array)
        detection_array = detectionArray()
        detection_array.scene_pc = self.input_pcd
        detection_array.depth_image = self.depth_image
        for i in range(len(all_detections)):
            #filter for objects in the whitelist only and object score
            if all_detections[i][1] < OBJECTNESS_THRESHOLD or (all_detections[i][0] not in WHITE_LIST):
                continue
            detection = votenetDetection()
            detection.semantic_class = all_detections[i][0]  # 0 : semantic class in string
            detection.object_score = all_detections[i][1]    # 1 : object score
            for j in range(len(all_detections[i][2])):       # 2 : bbox params (8x3)
                p = Point()
                p.x, p.y,p.z = all_detections[i][2][j][0],all_detections[i][2][j][1],all_detections[i][2][j][2]
                detection.bbox_3d.append(p)                 #append the point of BBOX
            detection_array.detections.append(detection)    #append BBOX to detections
        print('Publishing %d  values' %(len(detection_array.detections)))
        # publish this data to the desirable locations
        if len(detection_array.detections) > 0:
            self.publisher_.publish(detection_array)
        #publish the markers
        self.publishBoundingBox(all_detections)

if __name__ == '__main__':
    rospy.init_node('votenet_ros')
    votenet_ros = Votenet_ROS()
    rospy.spin()
    print('votenet node shutdown....')