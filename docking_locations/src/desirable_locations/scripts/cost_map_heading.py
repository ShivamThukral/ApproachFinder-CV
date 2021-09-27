#!/usr/bin/env python
import rospy
import numpy as np
from rviz_tools import RvizMarkers
from desirable_locations.msg import locationArray, desiredLocation, desirabilityGridMap
from scipy.stats import multivariate_normal
from scipy.special import logit, expit
import open3d as o3d
from convert_PCD import convertCloudFromOpen3dToRos,convertCloudFromRosToOpen3d
from sensor_msgs.msg import PointCloud2, PointField
import time
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Vector3, Quaternion
import math
from scipy.spatial.transform import Rotation as R
from joblib import Parallel, delayed

# ---------------------------------------------
# Optimisation: Comment the publish pcd part
#
#
#----------------------------------------------

frame = 'odom'
#frame = 'map'
#frame = 'world'
#frame = "camera_link"  #sjc dataset

#RVIZ PARAMS
RVIZ_DURATION = 4
PI = 3.14159
scale = Vector3(0.5, 0.05, 0.05) # x=length, y=height, z=height # single value for length (height is relative)
z_pi = R.from_euler('z', 180, degrees=True) # to visualise properly

#MAP DIMENSIONS
# map_bounds_x = np.array([-8.0,8.0],dtype = 'float')             # [x_min, x_max]
# map_bounds_y = np.array([-8.0,8.0],dtype = 'float')             # [x_min, x_max]
# map_bounds_theta = np.array([-3.14,3.14],dtype = 'float')       # [theta_min, theta_max]
# resolution = 0.025                                              # sampling in XY-plane


# map_bounds_x = np.array([-15.8,13.0],dtype = 'float') # [x_min,x_max]
# map_bounds_y = np.array([-15.8,13.0],dtype = 'float') # [x_min,x_max]
# resolution = 0.1

map_bounds_x = np.array([-10.0,9.2],dtype = 'float') # [x_min,x_max]
map_bounds_y = np.array([-10.0,9.2],dtype = 'float') # [x_min,x_max]
map_bounds_theta = np.array([0,2.0*PI],dtype = 'float')       # [theta_min, theta_max]
resolution = 0.1
HEADING_BINS = 4.0
theta_resolution = round(2.0*math.pi/HEADING_BINS,5)                         # sampling in radians


#grid initialisations with theta
grid_x, grid_y, grid_theta = np.mgrid[map_bounds_x[0]:map_bounds_x[1]:resolution, map_bounds_y[0]:map_bounds_y[1]:resolution, map_bounds_theta[0]:map_bounds_theta[1]:theta_resolution]
r,c,t = grid_x.shape

# this pcd is used for temporal desirability
temporal_desirability_xytheta = []      # used to store log-odds sum
grid_xytheta = []                       # used to calculate pdf values

temporal_desirability_xytheta = np.zeros((r,c,t),dtype='float')

grid_xytheta = np.empty(grid_x.shape + (3,))
grid_xytheta[:, :, :,0] = grid_x
grid_xytheta[:, :, :,1] = grid_y
grid_xytheta[:, :, :,2] = grid_theta

x_grid, y_grid = np.mgrid[map_bounds_x[0]:map_bounds_x[1]:resolution, map_bounds_y[0]:map_bounds_y[1]:resolution]
xyz = np.zeros((r*c, 3),dtype='float')
xyz[:, 0] = np.reshape(x_grid, -1)
xyz[:, 1] = np.reshape(y_grid, -1)

scalar = 0.9 # How much gaussian spread we want 0.1 m or 10 cm
cov_mat = scalar * np.identity(3, dtype = 'float')
cov_mat[2][2] = PI/18.0   # 45-degrees only
#cov_mat[2][2] = 0.174533    # 10-degree

def triVariatePDF(mu):
    """Return the multivariate Gaussian distribution on array pos.
    pos is an array constructed by packing the meshed arrays of variables
    x_1, x_2, x_3, ..., x_k into its _last_ dimension.
    """
    Sigma = cov_mat
    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)
    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2*np.pi)**n * Sigma_det)
    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
    # way across all the input variables.
    fac = np.einsum('...k,kl,...l->...', grid_xytheta-mu, Sigma_inv, grid_xytheta-mu)
    return np.exp(-fac / 2) / N

class costMap:
    def __init__(self):
        self.subscriber_ = rospy.Subscriber("/pcl_processing/desirable_locations",locationArray , self.callback, queue_size=1)
        self.markers = RvizMarkers(frame, '/pcl_processing/locations_rviz')
        self.pub_costmap = rospy.Publisher("/autorally_control/costmap", desirabilityGridMap, queue_size = 1)
        self.temporal_publisher = [rospy.Publisher("/desirability/temporal_3d_" + str(int(i*theta_resolution * 180/math.pi)) , PointCloud2, queue_size=1) for i in range(t)]
        self.instantaneous_publisher = [rospy.Publisher("/desirability/instant_3d_" + str(int(i*theta_resolution * 180/math.pi)) , PointCloud2, queue_size=1) for i in range(t)]
        self.occupany_costmap_pub = [rospy.Publisher("/desirablility/occupancy_grid_" + str(int(i*theta_resolution * 180/math.pi)), OccupancyGrid, queue_size = 1) for i in range(t)]
        self.unique_id = 0
        print('Subscriber and Publisher Set...')

    def publishLocations(self, all_locations):
        N = len(all_locations.desired_locations)
        points = []
        theta = np.empty(shape=(N,1),dtype='float')
        weights = np.empty(shape=(N,1),dtype='float')

        for i in range(N):
            points.append(all_locations.desired_locations[i].location)
            heading = all_locations.desired_locations[i].heading
            qr = R.from_quat([heading.x, heading.y, heading.z, heading.w])
            qr = qr * z_pi
            z_rot = qr.as_euler('zyx',degrees=False)[0]
            if z_rot < 0:
                z_rot += 2*math.pi;
            #theta[i] = qr.as_euler('zyx',degrees=False)[0]                             #only consider rotation about z-axis
            quat = Quaternion(qr.as_quat()[0],qr.as_quat()[1],qr.as_quat()[2],qr.as_quat()[3])
            P = Pose(points[i],quat)
            self.markers.publishArrow(P, 'yellow', scale, RVIZ_DURATION)                # pose, color, arrow_length, lifetime
            theta[i] = z_rot                                                           #only consider rotation about z-axis
            weights[i] = all_locations.desired_locations[i].location_weight

        self.markers.publishSpheres(points, 'green', 0.09, RVIZ_DURATION) # path, color, diameter, lifetime
        locations = np.array([[p.x,p.y,p.z] for p in points]) #unpack the point
        # assertion: each point should have one corresponding weight
        assert(locations.shape[0] == weights.shape[0])
        print('Published {} Locations....'.format(N))
        return locations, weights, theta

    def createGaussianFunctions(self, points, weights, theta):
        N = len(weights)
        points[:,2] = theta[:,0] #replace the last col (height) with desired heading.
        #tic = time.perf_counter()
        pdf_sum = np.zeros(shape=(r, c, t),dtype='float')
        F = [multivariate_normal(points[i], cov_mat) for i in range(N)]
        results = Parallel(n_jobs=4)(delayed(F[i].pdf)(grid_xytheta) for i in range(N))
        results = np.array(results)                 # N,R,C,T                        #convert this to np array for calculations
        print("Shape of Results ndarray = {}".format(results.shape))
        for i in range(N):
            pdf = np.multiply(results[i,:,:,:],weights[i])
            pdf_sum = np.add(pdf_sum, pdf)

        # normalise the wholw map for 0-1 probability
        normalise = np.amax(pdf_sum[:,:,:])
        for i in range(t):
            pdf_sum[:,:,i] = pdf_sum[:,:,i] if normalise == 0 else np.divide(pdf_sum[:,:,i], normalise)
        #toc = time.perf_counter()
        #print(f" Function+++++++++++++++++++++++ time {toc - tic:0.4f} seconds")

        # Visualise this grid to check if everything is fine
        for i in range(t):
            xyz[:, 2] = pdf_sum[:,:,i].flatten()
            open3d_pcd = o3d.geometry.PointCloud()
            open3d_pcd.points = o3d.utility.Vector3dVector(xyz)
            ros_cloud = convertCloudFromOpen3dToRos(open3d_pcd,frame)
            self.instantaneous_publisher[i].publish(ros_cloud)
        # results = Parallel(n_jobs=4)(delayed(triVariatePDF)(points[i]) for i in range(N))
        # for i in range(N):
        #     pdf = np.multiply(results[i],weights[i])
        #     pdf_sum = np.add(pdf_sum, pdf)          # this is element-wise so it works
        # #toc = time.perf_counter()
        # #noramlise along each heading separately
        # for i in range(angles):
        #     normalise = np.amax(pdf_sum[:,:,i])
        #     pdf_sum[:,:,i] = pdf_sum[:,:,i] if normalise == 0 else np.divide(pdf_sum[:,:,i], normalise)
        # #normalise = np.amax(pdf_sum)
        #pdf_sum = pdf_sum if normalise == 0 else np.divide(pdf_sum, normalise)
        #print(f" Function+++++++++++++++++++++++ time {toc - tic:0.4f} seconds")
        #Visualise this grid to check if everything is fine
        # xyz[:, 2] = np.reshape(pdf_sum[:,:,1],-1)
        # #Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
        # pcd_plane = o3d.geometry.PointCloud()
        # pcd_plane.points = o3d.utility.Vector3dVector(xyz)
        # #o3d.visualization.draw_geometries([pcd_plane])
        # ros_cloud = convertCloudFromOpen3dToRos(pcd_plane,frame)
        # self.pub_.publish(ros_cloud)
        print("Instantaneous done!")
        return  pdf_sum

    def computeTemporalDesirability(self, instant_desirable):
        global temporal_desirability_xytheta
        # calculations for current sensor measurement

        inverse_sensor_measurement = logit(instant_desirable)
        #inverse_sensor_measurement[inverse_sensor_measurement > 5.0] = 5.0
        #inverse_sensor_measurement[inverse_sensor_measurement < -5.0] = -5.0
        # temporal_desirability_xytheta = inverse_sensor_measurement
        temporal_desirability_xytheta = np.add(temporal_desirability_xytheta,inverse_sensor_measurement)
        temporal_desirability_xytheta[temporal_desirability_xytheta > 5.0] = 5.0
        temporal_desirability_xytheta[temporal_desirability_xytheta < -5.0] = -5.0
        # temporal_desirability_xytheta = inverse_sensor_measurement


        # for plotting the pcd convert it back
        for i in range(t):
            xyz[:,2] = expit(temporal_desirability_xytheta[:,:,i]).flatten()   # do element wise expit to get back the belief
            print(" {} = {} = {} ".format(i,int(i*theta_resolution * 180/math.pi),np.amax(xyz[:,2])))
            #uncomment to publish desirable temporal map
            desirable_pcd =  o3d.geometry.PointCloud()
            desirable_pcd.points = o3d.utility.Vector3dVector(xyz)
            ros_cloud = convertCloudFromOpen3dToRos(desirable_pcd,frame)
            self.temporal_publisher[i].publish(ros_cloud)
        print("Temporal Done..")

    def publishDesirableLocationsCostmap(self):
        xyz_points = expit(temporal_desirability_xytheta)  # since this was in log-odds
        # for each bin heading we need one occupancy grid
        desirability_maps = desirabilityGridMap()
        for i in range(t):
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
            # header
            costmap.header.seq = self.unique_id
            self.unique_id += 1
            costmap.header.stamp = rospy.Time.now()
            costmap.header.frame_id = frame
            # meta-data
            costmap.info = meta_data
            #faster data conversions
            #faster data conversions
            data = np.reshape(xyz_points[:,:,i],(meta_data.width, meta_data.height))
            data = np.multiply(data,100.0)
            data = np.transpose(data).flatten()
            data = data.astype('int64')
            costmap.data = data
            desirability_maps.desirability_maps.append(costmap)
            self.occupany_costmap_pub[i].publish(costmap)
        self.pub_costmap.publish(desirability_maps)


    def callback(self, all_locations):
        # publishes the parking spots
        points, weights, headings = self.publishLocations(all_locations)
        tic = time.perf_counter()
        instant_desirable = self.createGaussianFunctions(points, weights, headings)
        # pass these values for temporal desirability
        self.computeTemporalDesirability(instant_desirable)
        self.publishDesirableLocationsCostmap()
        toc = time.perf_counter()
        print(f" Function time {toc - tic:0.4f} seconds")


if __name__ == '__main__':
    rospy.init_node('costmap_heading')
    cost_map = costMap()
    rospy.spin()
    print('cost_map_heading node shutdown....')
