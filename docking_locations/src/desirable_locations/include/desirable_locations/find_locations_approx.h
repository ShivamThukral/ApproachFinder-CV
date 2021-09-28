//
// Created by vcr on 2020-12-10.
//

#ifndef SIMULATION_WS_FIND_LOCATIONS_APPROX_H
#define SIMULATION_WS_FIND_LOCATIONS_APPROX_H


#include "iostream"
#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include "desirable_locations/detectionArray.h"
#include "desirable_locations/votenetDetection.h"
#include "desirable_locations/desiredLocation.h"
#include "desirable_locations/locationArray.h"
#include "desirable_locations/o3dViz.h"
#include <geometry_msgs/Point.h>
#include "algorithm"
#include <math.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <limits>
#include <geometry_msgs/Transform.h>
#include <boost/thread/mutex.hpp>
#include <pcl/filters/passthrough.h>

#define PADDING_OFFSET 1.0  // 80cm away form table
#define WHEELCHAIR_SAMPLING_DISTANCE 0.05  // sampling at every 5 cm
#define WHEELCHAIR_WIDTH 0.7
#define WHEELCHAIR_LENGTH 0.7
#define WHEELCHAIR_DEPTH 0.8
#define PI 3.14159265
#define CLUSTER_TOLERANCE WHEELCHAIR_WIDTH/2.0 //25 cm - this is wheelchair length/2
#define MIN_CLUSTER_SIZE 1
#define DECREASE_RANGE 0.124  // 1/4th of wheelchair width


//#define VISUALISATIONS 1
#define EUCLIDEAN_DISTANCE_THRESHOLD 0.05  // 10cms
#define EUCLIDEAN_CLUSTER_MIN_SIZE 120

//--------------------------------------------------------------------------------
#define OBJECTNESS_THRESHOLD 0.7          //tables above this threshold are only considered for plane fitting
#define NUM_OF_POINTS_FOR_OVERLAPP 10               // number of points required to considered an overlap of two BB
#define PLANE_DISTANCE_THRESHOLD 0.02       // Points above and below 2 cms are considered as table top
#define STATISTICAL_OUTLIER_NEIGHBOURS 50    // The number of neighbors to analyze for each point is set to 50
#define STATISTICAL_OUTLIER_STD 2            // Standard Deviation multiplier is 2 - all points who have distance larger than 2 std of the mean distance to the query point is marked as outlier
#define REGION_GROWING_NEIGHBOURS 30
#define REGION_GROWING_SMOOTHNESS 3.0 / 180.0 * M_PI  // angle in radians that will be used as the allowable range for the normals deviation
#define REGION_GROWING_CURVATURE 1.0                    //curvature threshold
#define REGION_GROWING_OVERLAPPING 30

#define OVERLAP_POINTS 50 // the points box and cluster should overlap to be considered.

//--------------------------------------------------------------------------------

#ifndef NDEBUG
#   define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#else
#   define ASSERT(condition, message) do { } while (false)
#endif

#ifdef VISUALISATIONS
//visualisations
pcl::visualization::PCLVisualizer::Ptr viewer_approx(new pcl::visualization::PCLVisualizer("3D Viewer"));

#endif

ros::Publisher pub_approx,pub_table_approx, pub_o3d;
sensor_msgs::CameraInfo cam_info;
cv::Mat normalized;
//simulation
std::string src_frame = "odom";
std::string des_frame = "camera_depth_optical_frame";

// dataset
//std::string src_frame = "world";
//std::string des_frame = "openni_depth_optical_frame";

//sjc dataset
//std::string src_frame = "camera_link";
//std::string des_frame = "camera_depth_optical_frame";
boost::mutex access_guard_;

class Open3DViz
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_top_cloud;
    std::vector <Eigen::Vector3f> polygon_points;
    std::vector < std::vector <Eigen::Vector3f>> potential_placements;
    std::vector <std::vector<Eigen::Vector3f>> all_clusters;  //all the clusters from all the table detections
    std::vector <std::vector<double>> all_weights  ;             // all the weights from all the detections.
    std::vector <std::vector<Eigen::Quaternionf>> all_heading;   // heading for all detections
    cv::Mat depth_image;
    void publishData();
};

class DetectTableApprox {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud;
    cv::Mat depth_image;
    std::vector <std::vector<Eigen::Vector3f>> all_clusters;  //all the clusters from all the table detections
    std::vector <std::vector<double>> all_weights  ;             // all the weights from all the detections.
    std::vector <std::vector<Eigen::Quaternionf>> all_heading;   // heading for all detections
    ros::NodeHandle nh_;
    int marker_id;
    Open3DViz *o3dviz;

public:
    //constructor
    DetectTableApprox(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, cv::Mat &d_image,
                std::vector <desirable_locations::votenetDetection> detections);
    //destructor
    ~DetectTableApprox();

    //transform listener
    geometry_msgs::TransformStamped listenTransform(std::string des_frame,std::string src_frame);

    // helper function for weights calculation using clustering
    std::vector <std::vector<Eigen::Vector3f>>
    makeClusters(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations, std::vector<int> &along_edge);

    // trapezium function for weights
    std::vector <std::vector<double>> calculatePositionalWeights(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,
                                                       std::vector <std::vector<Eigen::Vector3f>> &clusters);
    std::vector <std::vector<double>>
    findPositionalWeights(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations, std::vector <std::vector<Eigen::Vector3f>> &clusters);

    std::vector <std::vector<double>> calculateVisibilityWeights(std::vector <std::vector<Eigen::Vector3f>> &locations,std::vector<std::vector<Eigen::Quaternionf>> &heading, std::vector<double> &wheelchair_dimensions, geometry_msgs::TransformStamped transformStamped);

    std::vector<std::vector<std::pair<double,double>>> projectOnDepthImage(std::vector <std::vector<Eigen::Vector3f>> locations, std::vector<double> wheelchair_dimensions);

    // filter for collision and get collision free placements of the wheelchair
    std::vector <std::vector<Eigen::Vector3f>> filterForCollision(std::vector <std::vector<Eigen::Vector3f>> &locations,
                                                                  std::vector<double> &wheelchair_dimensions);

    //filter for points that are inside the FOV
    std::vector<std::vector<Eigen::Vector3f>> filterForFOV(std::vector <std::vector<Eigen::Vector3f>> &locations, geometry_msgs::TransformStamped transformStamped);

    std::vector<Eigen::Vector3f> calculatePaddedLines(std::vector<Eigen::Vector3f> &approx_polygon,double padding_offset);

    // find all locations around the table - with collisions
    std::vector <std::vector<Eigen::Vector3f>>
    findPossiblePlacements(std::vector <Eigen::Vector3f> &approx_polygon,
                                    double padding_offset, double location_sampling_distance);


    // 2d rectangle for table top on the PCD data
    std::vector<Eigen::Vector3f>
    findMinimumAreaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void publishTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd);

    pcl::PointCloud<pcl::PointXYZ>::Ptr findTableTop(pcl::ModelCoefficients::Ptr plane_coefficients,  pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd);

    // fits a plane onto table top and returns the plane coefficients
    void
    fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr table_bbox_cloud, pcl::ModelCoefficients::Ptr coefficients);

    //extracts the PCD given 8 corners of the bbox
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractBBoxPCD(std::vector <geometry_msgs::Point> &bbox_corners);

    //void publishDesirableLocations(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,double rotation_radian);
    void  publishLocations(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations, std::vector <std::vector<double>> &weights, std::vector<std::vector<Eigen::Quaternionf>> &heading);

    std::map<int,std::vector<int>> findConnectedComponents(std::map<int,std::vector<int>> &intersections);
    void DFS(std::map<int,std::vector<int>> &intersections, std::set<int> &visited,std::vector<int> &path,int node);

    //given interesction value
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> computeOverlap(std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> &extracted_pcd);

    //finds intersections between the bounding boxes and returns as map
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> findIntersectingBoundingBoxes(std::vector <desirable_locations::votenetDetection> &detections);

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPCD(sensor_msgs::Image depth_msg);

    std::vector<std::vector<Eigen::Quaternionf>> calculateHeading(std::vector<std::vector<Eigen::Vector3f>> &locations);

    std::vector<Eigen::Vector3f> addInbetweenPaddedLines(std::vector<Eigen::Vector3f> padded_approx_polygon);

    void simpleVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string title) {
        // --------------------------------------------
        // -----Open 3D viewer xyz and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer_approx(new pcl::visualization::PCLVisualizer(title));
        viewer_approx->setBackgroundColor(0, 0, 0);
        viewer_approx->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
        viewer_approx->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer_approx->addCoordinateSystem(1.0);
        viewer_approx->initCameraParameters();
        while (!viewer_approx->wasStopped()) {
            viewer_approx->spinOnce(100);
            sleep(0.1);
        }
    }
};


#endif //SIMULATION_WS_FIND_LOCATIONS_APPROX_H
