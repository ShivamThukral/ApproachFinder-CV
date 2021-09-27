#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from rviz_tools import RvizMarkers
from desirable_locations.msg import locationArray, desiredLocation
from scipy.stats import multivariate_normal
from scipy.special import logit, expit
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import ros_numpy
from convert_PCD import convertCloudFromOpen3dToRos,convertCloudFromRosToOpen3d
from sensor_msgs.msg import PointCloud2, PointField
import time
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Vector3, Quaternion
import math
from scipy.spatial.transform import Rotation as R
import multiprocessing as mp
from joblib import Parallel, delayed





# ---------------------------------------------
# Optimisation: Comment the publish pcd part
#
#
#----------------------------------------------
# this pcd is used for temporal desirability
temporal_desirability_xyz  = []
grid_pos = []
initialise_values = False
frame = 'odom'
#frame = 'map'
#frame = 'world'

#sjc dataset
#frame = "camera_link"
scalar = 0.2 # How much gaussian spread we want 0.1 m or 10 cm
map_spread = 4.0
RVIZ_DURATION = 4.0
# map_bounds_x = np.array([-8.0,8.0],dtype = 'float') # [x_min,x_max]
# map_bounds_y = np.array([-8.0,8.0],dtype = 'float') # [x_min,x_max]
# resolution = 0.025


# map_bounds_x = np.array([-15.8,13.0],dtype = 'float') # [x_min,x_max]
# map_bounds_y = np.array([-15.8,13.0],dtype = 'float') # [x_min,x_max]
# resolution = 0.1

map_bounds_x = np.array([-10.0,9.2],dtype = 'float') # [x_min,x_max]
map_bounds_y = np.array([-10.0,9.2],dtype = 'float') # [x_min,x_max]
resolution = 0.1

# grid initialisations:
x_grid, y_grid = np.mgrid[map_bounds_x[0]:map_bounds_x[1]:resolution, map_bounds_y[0]:map_bounds_y[1]:resolution]
temporal_desirability_xyz = np.zeros((np.size(x_grid), 3))
temporal_desirability_xyz [:, 0] = np.reshape(x_grid, -1) # x-values of the grid
temporal_desirability_xyz [:, 1] = np.reshape(y_grid, -1) # y-values of the grid
# create grid which will be used by pdf functions
grid_pos = np.empty(x_grid.shape + (2,))
grid_pos[:, :, 0] = x_grid
grid_pos[:, :, 1] = y_grid
#scalar = how much spread we want
covariance_matrix = scalar * np.identity(2, dtype = 'float')





def calculatePDF(F,W,d):
    d[0] = np.add( np.multiply(F.pdf(grid_pos),W), d[0])

def biVariatePDF(mu):
    """Return the multivariate Gaussian distribution on array pos.
    pos is an array constructed by packing the meshed arrays of variables
    x_1, x_2, x_3, ..., x_k into its _last_ dimension.
    """
    Sigma = covariance_matrix
    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)
    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2*np.pi)**n * Sigma_det)
    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
    # way across all the input variables.
    fac = np.einsum('...k,kl,...l->...', grid_pos-mu, Sigma_inv, grid_pos-mu)
    return np.exp(-fac / 2) / N

class costMap:
    def __init__(self):
        self.subscriber_ = rospy.Subscriber("/pcl_processing/desirable_locations",locationArray , self.callback, queue_size=1)
        self.markers = RvizMarkers(frame, '/pcl_processing/locations_rviz')
        self.pub_ = rospy.Publisher("/desirability/instantaneous", PointCloud2, queue_size=1)
        self.pub_temporal = rospy.Publisher("/desirability/temporal", PointCloud2, queue_size=1)
        self.pub_costmap = rospy.Publisher("/desirable_locations/costmap", OccupancyGrid, queue_size = 1)
        self.unique_id = 0
        print('Subscriber and Publisher Set...')

    def publishLocations(self, all_locations):
        N = len(all_locations.desired_locations)
        points = []
        scale = Vector3(0.5,0.05,0.05) # x=length, y=height, z=height # single value for length (height is relative)
        for i in range(N):
            points.append(all_locations.desired_locations[i].location)
            q = [all_locations.desired_locations[i].heading.x, all_locations.desired_locations[i].heading.y,
                 all_locations.desired_locations[i].heading.z, all_locations.desired_locations[i].heading.w]
            qr = R.from_quat(q)
            r = R.from_euler('z', 180, degrees=True) # to visualise properly
            qr = qr * r
            quat = Quaternion(qr.as_quat()[0],qr.as_quat()[1],qr.as_quat()[2],qr.as_quat()[3] )
            P = Pose(points[i],quat)
            self.markers.publishArrow(P, 'yellow', scale, RVIZ_DURATION) # pose, color, arrow_length, lifetime
        self.markers.publishSpheres(points, 'green', 0.09, RVIZ_DURATION) # path, color, diameter, lifetime
        print('Published {} Locations....'.format(N))

    def calculateGrid(self):
        #calculate the bounds of the grid
        # x_bounds = np.array([0,0],dtype = 'double') # [x_min,x_max]
        # y_bounds = np.array([0,0],dtype = 'double') # [y_min,y_max]
        # x_bounds[0] = np.min(possible_placements[:,0]) - map_spread*scalar
        # x_bounds[1] = np.max(possible_placements[:,0]) + map_spread*scalar
        # y_bounds[0] = np.min(possible_placements[:,1]) - map_spread*scalar
        # y_bounds[1] = np.max(possible_placements[:,1]) + map_spread*scalar
        # #print(x_bounds)
        # #print(y_bounds)
        # x_grid, y_grid = np.mgrid[x_bounds[0]:x_bounds[1]:0.02, y_bounds[0]:y_bounds[1]:0.02]
        grid_x, grid_y, grid_heading = np.mgrid[map_bounds_x[0]:map_bounds_x[1]:resolution, map_bounds_y[0]:map_bounds_y[1]:resolution,0:6.283:0.785398]
        x_grid, y_grid = np.mgrid[map_bounds_x[0]:map_bounds_x[1]:resolution, map_bounds_y[0]:map_bounds_y[1]:resolution]
        return x_grid,y_grid

    def multivariate_gaussian(self, mu, Sigma):
        """Return the multivariate Gaussian distribution on array pos.
        pos is an array constructed by packing the meshed arrays of variables
        x_1, x_2, x_3, ..., x_k into its _last_ dimension.
        """
        n = mu.shape[0]
        Sigma_det = np.linalg.det(Sigma)
        Sigma_inv = np.linalg.inv(Sigma)
        N = np.sqrt((2*np.pi)**n * Sigma_det)
        # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
        # way across all the input variables.
        fac = np.einsum('...k,kl,...l->...', grid_pos-mu, Sigma_inv, grid_pos-mu)
        return np.exp(-fac / 2) / N


    def createGaussianFunctions(self,all_locations):
        N = len(all_locations.desired_locations)
        points = np.zeros(shape=(N,3))
        weights = np.zeros(shape=(N,1))
        for i in range(N):
            points[i][0] = (all_locations.desired_locations[i].location.x)
            points[i][1] = (all_locations.desired_locations[i].location.y)
            points[i][2] = (all_locations.desired_locations[i].location.z)
            weights[i] = (all_locations.desired_locations[i].location_weight)

        # assertion: each point should have one corresponding weight
        assert(points.shape[0] == weights.shape[0])
        # make a multivariate normal for each center location
        # F = [multivariate_normal(points[i,0:2], covariance_matrix) for i in range(N)]

        # rows, cols = grid_pos.shape[0], grid_pos.shape[1]
        # tic = time.perf_counter()
        # pdf_sum = np.zeros(shape=(rows,cols),dtype='float')
        # for i in range(N):
        #     pdf = np.multiply(F[i].pdf(grid_pos),weights[i])
        #     pdf_sum = np.add(pdf_sum, pdf)
        # toc = time.perf_counter()
        # normalise = np.amax(pdf_sum)
        # pdf_sum = pdf_sum if normalise == 0 else np.divide(pdf_sum, normalise)
        # print(f" Function**** time {toc - tic:0.4f} seconds")

        # tic = time.perf_counter()
        # rows, cols = grid_pos.shape[0], grid_pos.shape[1]
        # pdf_sum = np.zeros(shape=(rows,cols),dtype='float')
        # for i in range(N):
        #     mu = points[i,0:2]
        #     Z = self.multivariate_gaussian(mu, covariance_matrix)
        #     pdf = np.multiply(Z,weights[i])
        #     pdf_sum = np.add(pdf_sum, pdf)
        # toc = time.perf_counter()
        # normalise = np.amax(pdf_sum)
        # pdf_sum = pdf_sum if normalise == 0 else np.divide(pdf_sum, normalise)
        # print(f" Function------------------------ time {toc - tic:0.4f} seconds")

        #tic = time.perf_counter()
        rows, cols = grid_pos.shape[0], grid_pos.shape[1]
        pdf_sum = np.zeros(shape=(rows,cols),dtype='float')
        results = Parallel(n_jobs=4)(delayed(biVariatePDF)(points[i,0:2]) for i in range(N))
        for i in range(N):
            pdf = np.multiply(results[i],weights[i])
            pdf_sum = np.add(pdf_sum, pdf)
        #toc = time.perf_counter()
        normalise = np.amax(pdf_sum)
        pdf_sum = pdf_sum if normalise == 0 else np.divide(pdf_sum, normalise)
        #print(f" Function+++++++++++++++++++++++ time {toc - tic:0.4f} seconds")

        # uncomment this to visualise this the pcd
        #Visualise this grid to check if everything is fine
        xyz = np.zeros((np.size(x_grid), 3))
        xyz[:, 0] = np.reshape(x_grid, -1)
        xyz[:, 1] = np.reshape(y_grid, -1)
        xyz[:, 2] = np.reshape(pdf_sum,-1)
        #Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
        pcd_plane = o3d.geometry.PointCloud()
        pcd_plane.points = o3d.utility.Vector3dVector(xyz)
        # o3d.visualization.draw_geometries([pcd_plane])
        ros_cloud = convertCloudFromOpen3dToRos(pcd_plane,frame)
        self.pub_.publish(ros_cloud)

        print("Instantaneous done!")
        return  pdf_sum

   # def computeInverseSensorMeasurement(self, F, weights, pos, V):
    def computeInverseSensorMeasurement(self, V):
        # weight_len = len(weights)
        # rows,cols = pos.shape[0], pos.shape[1]
        # Z = np.empty([weight_len,rows,cols],dtype='double')
        # for i in range(weight_len):
        #     pdf = np.array(F[i].pdf(pos))    #(rows,cols)
        #     W = np.full((rows,cols),weights[i])
        #     Z[i] = pdf*W
        # V = np.sum(Z,axis=0)
        # N = np.amax(V)
        p_x1_zt = V
        # REMEMBER TO ADD TURE DIVIDE
        # Here Z values can be greater than 1 so apply max function
        #p_x1_zt = V/np.amax(V)  # V(i,j) = p(m[ij] = desirable| measurements)  <-- inverse measurement model
        inverse_measurement_model = logit(p_x1_zt)
        #reshape this for xyz_points
        inverse_measurement_model = np.reshape(inverse_measurement_model, -1)
        return inverse_measurement_model

    def computeTemporalDesirability(self, V):
        global temporal_desirability_xyz,grid_pos,initialise_values
        # if initialise_values == False:
        #     #create an empty desirability - prior is zero
        #     #x_grid, y_grid = np.mgrid[-1.0:4.0:0.05, -4.5:0.5:0.05]
        #     x_grid, y_grid = self.calculateGrid()
        #     temporal_desirability_xyz = np.zeros((np.size(x_grid), 3))
        #     temporal_desirability_xyz [:, 0] = np.reshape(x_grid, -1) # x-values of the grid
        #     temporal_desirability_xyz [:, 1] = np.reshape(y_grid, -1) # y-values of the grid
        #     # create grid which will be used by pdf functions
        #     grid_pos = np.empty(x_grid.shape + (2,))
        #     grid_pos[:, :, 0] = x_grid
        #     grid_pos[:, :, 1] = y_grid
        #     initialise_values = True
        #     #temporal_pcd = temporal_pcd.voxel_down_sample(voxel_size=0.05)

        # calculations for current sensor measurement
        log_odd_ratio = self.computeInverseSensorMeasurement(V)
        temporal_desirability_xyz[:,2]  = np.add(temporal_desirability_xyz[:,2],log_odd_ratio)  # add element-wise

        # for plotting the pcd convert it back
        xyz_points = np.asarray(temporal_desirability_xyz)
        xyz_points[:,2] = expit(xyz_points[:,2])   # do element wise expit to get back the belief
        #uncomment to publish desirable temporal map
        desirable_pcd =  o3d.geometry.PointCloud()
        desirable_pcd.points = o3d.utility.Vector3dVector(xyz_points)
        ros_cloud = convertCloudFromOpen3dToRos(desirable_pcd,frame)
        self.pub_temporal.publish(ros_cloud)
        # publish the map as occupancy grid to autorally code
        self.publishDesirableLocationsCostmap(xyz_points)
        print("Temporal Done..")

    def publishDesirableLocationsCostmap(self, xyz_points):
        costmap = OccupancyGrid()
        #meta-data
        meta_data = MapMetaData()
        meta_data.map_load_time = rospy.Time.now()
        meta_data.resolution = resolution
        meta_data.width = int(math.fabs(map_bounds_x[0] - map_bounds_x[1])*math.pow(resolution,-1.0))
        meta_data.height = int(math.fabs(map_bounds_y[0] - map_bounds_y[1])*math.pow(resolution,-1.0))
        meta_data.origin.position.x = map_bounds_x[0]
        meta_data.origin.position.y = map_bounds_y[0]
        r = R.from_euler('z', 0, degrees=True)
        meta_data.origin.orientation.x = r.as_quat()[0]
        meta_data.origin.orientation.y = r.as_quat()[1]
        meta_data.origin.orientation.z = r.as_quat()[2]
        meta_data.origin.orientation.w = r.as_quat()[3]
        #header
        costmap.header.seq = self.unique_id
        self.unique_id += 1
        costmap.header.stamp = rospy.Time.now()
        costmap.header.frame_id = frame
        # data
        costmap.info = meta_data
        #faster data conversions
        data = np.reshape(xyz_points[:,2],(meta_data.width, meta_data.height))
        data = np.multiply(data,100.0)
        data = np.transpose(data).flatten()
        data = data.astype('int64')
        # previous code slower version
        # data = [int(data*100) for data in xyz_points[:,2].flatten('C')]
        # data = np.reshape(data,(meta_data.width, meta_data.height))
        # data = np.transpose(data).flatten()
        costmap.data = data
        self.pub_costmap.publish(costmap)


    def callback(self, all_locations):
        # publishes the parking spots
        self.publishLocations(all_locations)
        tic = time.perf_counter()
        V = self.createGaussianFunctions(all_locations)
        toc = time.perf_counter()
        # pass the values for temporal desirability
        self.computeTemporalDesirability( V)
        print(f" Function time {toc - tic:0.4f} seconds")


if __name__ == '__main__':
    rospy.init_node('cost_map_calculations')
    cost_map = costMap()
    rospy.spin()
    print('cost_map node shutdown....')
