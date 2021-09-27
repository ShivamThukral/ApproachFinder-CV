#!/usr/bin/env python
import rospy
import numpy as np
from desirable_locations.msg import o3dViz, locationArray
from sensor_msgs.msg import PointCloud2
from convert_PCD import convertCloudFromOpen3dToRos,convertCloudFromRosToOpen3d
from rviz_tools import RvizMarkers
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Vector3, Quaternion
from scipy.stats import multivariate_normal
from joblib import Parallel, delayed
import open3d as o3d

RVIZ_DURATION = 2.0

class o3dVisualiser(object):
    def __init__(self, confid_dict):
        self.sub = rospy.Subscriber("/o3d/viz",  o3dViz, self.callback, queue_size=1)
        self.pub_scene = rospy.Publisher("/o3d/scene_pc", PointCloud2, queue_size = 1)
        self.pub_obj_top = rospy.Publisher("/o3d/obj_top_pc", PointCloud2, queue_size = 1)
        self.marker_poly_pts = RvizMarkers('odom', '/o3d/ply_pts')
        self.marker_pot_pts = RvizMarkers('odom', '/o3d/pot_pts')
        self.marker_loc = RvizMarkers('odom', '/o3d/loc')
        self.marker_arrow = RvizMarkers('odom', '/o3d/heading')
        self.inst_pub = rospy.Publisher("/o3d/instant_3d" , PointCloud2, queue_size=1)
        self.config_dict = config_dict

    def calculateGrid(self, points):
        x_bounds = np.array([0,0],dtype = 'double') # [x_min,x_max]
        y_bounds = np.array([0,0],dtype = 'double') # [y_min,y_max]
        map_spread = self.config_dict['map_spread']
        scalar = self.config_dict['scalar']
        resolution = self.config_dict['resolution']
        x_bounds[0] = np.min(points[:,0]) - map_spread*scalar
        x_bounds[1] = np.max(points[:,0]) + map_spread*scalar
        y_bounds[0] = np.min(points[:,1]) - map_spread*scalar
        y_bounds[1] = np.max(points[:,1]) + map_spread*scalar
        x_grid, y_grid = np.mgrid[x_bounds[0]:x_bounds[1]:resolution, y_bounds[0]:y_bounds[1]:resolution]
        r,c = x_grid.shape
        xyz = np.zeros((r*c, 3),dtype='float')
        xyz[:, 0] = np.reshape(x_grid, -1)
        xyz[:, 1] = np.reshape(y_grid, -1)
        grid_pos = np.empty(x_grid.shape + (2,))
        grid_pos[:, :, 0] = x_grid
        grid_pos[:, :, 1] = y_grid
        return grid_pos, xyz

    def generateGaussian(self,  actual_points, weights, theta, grid_xy):
        r,c,_ = grid_xy.shape
        pdf_sum = np.zeros(shape=(r, c),dtype='float')
        N = len(weights)
        points = np.array(actual_points)

        cov_mat = self.config_dict['scalar'] * np.identity(2, dtype = 'float')
        F = [multivariate_normal(points[i,0:2], cov_mat) for i in range(N)]

        results = Parallel(n_jobs=4)(delayed(F[i].pdf)(grid_xy) for i in range(N))
        results = np.array(results)                 # N,R,C                        #convert this to np array for calculations
        for i in range(N):
            pdf = np.multiply(results[i,:,:],weights[i])
            pdf_sum = np.add(pdf_sum, pdf)
        # normalise the wholw map for 0-1 probability
        normalise = np.amax(pdf_sum)
        pdf_sum = pdf_sum if normalise == 0 else np.divide(pdf_sum, normalise)
        return pdf_sum

    def callback(self, msg):
        print('recived...')
        xyz = convertCloudFromRosToOpen3d(msg.scene_pc)
        xyz_ros = convertCloudFromOpen3dToRos(xyz)
        self.pub_scene.publish(xyz_ros)
        xyz_obj = convertCloudFromRosToOpen3d(msg.obj_top_pc)
        xyz_obj_ros = convertCloudFromOpen3dToRos(xyz_obj)
        self.pub_obj_top.publish(xyz_obj_ros)
        self.marker_poly_pts.publishSpheres(msg.poly_pts, 'green', 0.12, RVIZ_DURATION) # path, color, diameter, lifetime
        self.marker_pot_pts.publishSpheres(msg.pot_pts,'red',0.09, RVIZ_DURATION)
        self.marker_loc.publishSpheres(msg.loc,'blue',0.1, RVIZ_DURATION)
        theta=[]
        #publish arrows
        for i,heading in enumerate(msg.heading):
            qr = R.from_quat([heading.x, heading.y, heading.z, heading.w])
            z_pi = R.from_euler('z', 180, degrees=True) # to visualise properly
            qr = qr * z_pi
            z_rot = qr.as_euler('zyx',degrees=False)[0]
            if z_rot < 0:
                z_rot += 2*np.pi;
            theta.append(z_rot)
            quat = Quaternion(qr.as_quat()[0],qr.as_quat()[1],qr.as_quat()[2],qr.as_quat()[3])
            P = Pose(msg.loc[i],quat)
            scale = Vector3(0.3, 0.02, 0.02) # x=length, y=height, z=height # single value for length (height is relative)
            if i%2 == 0:
                self.marker_arrow.publishArrow(P, 'red', scale, RVIZ_DURATION)


        points = np.array([[p.x, p.y, p.z] for p in msg.loc])
        weight = np.array(msg.location_weight)
        theta = np.array(theta)
        grid_xy, xyz = self.calculateGrid(points)
        pdf_sum = self.generateGaussian(points, weight, theta, grid_xy)
        xyz[:, 2] = np.reshape(pdf_sum,-1)
        open3d_pcd = o3d.geometry.PointCloud()
        open3d_pcd.points = o3d.utility.Vector3dVector(xyz)
        ros_cloud = convertCloudFromOpen3dToRos(open3d_pcd,'odom')
        self.inst_pub.publish(ros_cloud)







if __name__ == '__main__':
    rospy.init_node('o3d_viz')
    config_dict = { 'scalar' : 0.2, 'map_spread' : 10.0, 'resolution' : 0.02 }
    viz = o3dVisualiser(config_dict)
    rospy.spin()
    print('visualiser shutdown....')