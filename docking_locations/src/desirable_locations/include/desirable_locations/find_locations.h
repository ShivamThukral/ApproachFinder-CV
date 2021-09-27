//
// Created by vcr on 2020-10-29.
//

#ifndef SIMULATION_WS_FIND_LOCATIONS_H
#define SIMULATION_WS_FIND_LOCATIONS_H

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
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include "desirable_locations/detectionArray.h"
#include "desirable_locations/votenetDetection.h"
#include "desirable_locations/desiredLocation.h"
#include "desirable_locations/locationArray.h"
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


#define PADDING_OFFSET 0.8  // 80cm away form table
#define WHEELCHAIR_SAMPLING_DISTANCE 0.05  // sampling at every 5 cm
#define WHEELCHAIR_WIDTH 0.5
#define WHEELCHAIR_LENGTH 0.5
#define WHEELCHAIR_DEPTH 1
#define PI 3.14159265
#define CLUSTER_TOLERANCE 0.25 //25 cm
#define MIN_CLUSTER_SIZE 3
#define DECREASE_RANGE 0.124  // 1/4th of wheelchair width
#define PLANE_DISTANCE_THRESHOLD 0.01
#define OBJECTNESS_THRESHOLD 0.85




#define VISUALISATIONS 1

#ifdef VISUALISATIONS
//visualisations
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
#endif

ros::Publisher pub_,pub_table;

class DetectTable {
private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud;
    std::vector <std::vector<Eigen::Vector3f>> all_clusters;  //all the clusters from all the table detections
    std::vector <std::vector<double>> all_weights  ;             // all the weights from all the detections.
    ros::NodeHandle nh_;
    // publish locations
    //For RVIZ Visualisations
//    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;


public:
    DetectTable(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,
                std::vector <desirable_locations::votenetDetection> detections);
    ~DetectTable();

    // helper function for weights calculation using clustering
    std::vector <std::vector<Eigen::Vector3f>>
    makeClusters(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations, std::vector<int> &along_edge);

    // trapezium function for weights
    std::vector <std::vector<double>> calculateWeights(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,
                                                                    std::vector <std::vector<Eigen::Vector3f>> &clusters);

    // filter for collision and get collision free placements of the wheelchair
    std::vector <std::vector<Eigen::Vector3f>> filterForCollision(std::vector <std::vector<Eigen::Vector3f>> &locations,
                                                                  std::vector<double> &wheelchair_dimensions);

    //filter for points that are inside the FOV
    std::vector<std::vector<Eigen::Vector3f>> filterForFOV(std::vector <std::vector<Eigen::Vector3f>> locations);

    // find all locations around the table - with collisions
    std::vector <std::vector<Eigen::Vector3f>>
    findPossiblePlacementsRectangle(std::vector <Eigen::Vector3f> padded_table_bbox,
                                                 double location_sampling_distance);

    std::vector <std::vector<Eigen::Vector3f>>
    findPossiblePlacementsCircle(Eigen::Vector3f circle_center, float radius,
                                    double location_sampling_distance);

    // 2d rectangle for table top on the PCD data
    void
    findMinimumAreaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double padding_offset,
                                      std::vector <Eigen::Vector3f> &table_top_bbx, std::vector <Eigen::Vector3f> &padded_table_top_bbx, double &rotation_radian,
                                      bool &isRectangle,Eigen::Vector3f &circle_center, float &radius);

    void publishTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd);

    pcl::PointCloud<pcl::PointXYZ>::Ptr findTableTop(pcl::ModelCoefficients::Ptr plane_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud);

    // fits a plane onto table top and returns the plane coefficients
    void
    fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr table_bbox_cloud, pcl::ModelCoefficients::Ptr coefficients);

    // extracts the PCD given 8 corners of the bbox
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractBBoxPCD(std::vector <geometry_msgs::Point> bbox_corners);

//    void publishDesirableLocations(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,double rotation_radian);
    void  publishLocations(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations, std::vector <std::vector<double>> &weights);

    std::map<int,std::vector<int>> findConnectedComponents(std::map<int,std::vector<int>>intersections);
    void DFS(std::map<int,std::vector<int>>intersections, std::set<int> &visited,std::vector<int> &path,int node);

    //given interesction value
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> computeOverlap(std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pcd);

    //finds intersections between the bounding boxes
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> findIntersectingBoundingBoxes(std::vector <desirable_locations::votenetDetection> detections);

    void simpleVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string title) {
        // --------------------------------------------
        // -----Open 3D viewer xyz and add point cloud-----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            sleep(0.1);
        }
    }
};

#endif //SIMULATION_WS_FIND_LOCATIONS_H
