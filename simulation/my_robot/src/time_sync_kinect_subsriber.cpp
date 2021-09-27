//
// Created by vcr on 2021-01-13.
//

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "my_robot/kinect_data.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


#define SUBSCRIBER_DELAY 5

//#define ZED_CAMERA 1
#define SIMULATION 1
//#define XTION 1

using namespace std;
bool subscribe_flag = true; // used to subscribe only once.
pcl::PointCloud<pcl::PointXYZ>::Ptr votenet_cloud(new pcl::PointCloud <pcl::PointXYZ>);
sensor_msgs::Image votenet_depth_image;

pcl::PCLPointCloud2::Ptr scene(new pcl::PCLPointCloud2());
sensor_msgs::Image recieved_depth_image;


ros::Publisher pub_,pub_rviz;


//simulation
#ifdef SIMULATION
    std::string des_frame = "odom";
    std::string src_frame = "camera_depth_optical_frame";
#endif

//dataset
//std::string des_frame = "world";
//std::string src_frame = "openni_depth_optical_frame";

#ifdef XTION
//sjc dataset -xtion
std::string des_frame = "camera_link";
std::string src_frame = "camera_depth_optical_frame";
#endif

//sjc dataset - zed2
#ifdef ZED_CAMERA
    std::string des_frame = "map";
    std::string src_frame = "zed2_left_camera_optical_frame";
#endif


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

/*
 * Depth image to cv::Mat
void imageVis(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(message->depth_image,message);

    // imshow expects a float value to lie in [0,1], so we need to normalize
    // for visualization purposes.
    double max = 0.0;
    cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
    cv::Mat normalized;
    cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0)  ;

    cv::imshow("foo", normalized);
    cv::waitKey(0);
}*/


geometry_msgs::TransformStamped listenTransform(std::string des_frame, std::string src_frame) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(des_frame.c_str(), src_frame.c_str(),
                                                    ros::Time(0),
                                                    ros::Duration(3.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform %s to %s: %s", des_frame.c_str(), src_frame.c_str(), ex.what());
    }
    return transformStamped;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropBounds(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_cloud, double x_min,double x_max, double y_min, double y_max, double z_min, double z_max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (unfiltered_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*cloud_filtered_xyz);
//    pass.setInputCloud (cloud_filtered_xyz);
//    pass.setFilterFieldName ("y");
//    pass.setFilterLimits (y_min, y_max);
//    pass.filter (*cloud_filtered_xyz);
//    pass.setInputCloud (cloud_filtered_xyz);
//    pass.setFilterFieldName ("x");
//    pass.setFilterLimits (x_min, x_max);
//    pass.filter (*cloud_filtered_xyz);
    return cloud_filtered_xyz;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterStatisticalOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr stat_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (20);
    sor.setStddevMulThresh (2.0);
    sor.filter (*stat_cloud_filtered);
    return stat_cloud_filtered;

}

void myCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr &input) {
//copy the data to global variables
    recieved_depth_image = *image;

    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
    sensor_msgs::PointCloud2 scene_world;
    tf2::doTransform(*input, scene_world, transformStamped);
    pcl_conversions::toPCL(scene_world,*scene);



  /*
    ros::Time start_time = ros::Time::now();
    // transformation the pcd to world frame
    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
    sensor_msgs::PointCloud2 scene_world;
    tf2::doTransform(*input, scene_world, transformStamped);
    pcl::PCLPointCloud2::Ptr scene(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(scene_world,*scene);
    //ROS_INFO("Transformed Scene Cloud: width = %d, height = %d\n", scene->width, scene->height);

    // Create the filtering object
    std::vector<float> leaf_size = {0.01f, 0.01f, 0.01f};
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr scene_voxelised(new pcl::PCLPointCloud2());
    sor.setInputCloud(scene);
    sor.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    sor.filter(*scene_voxelised);
    //ROS_INFO("Scene Voxelized: width = %d, height = %d\n", scene_voxelised->width, scene_voxelised->height);

    pcl::PointCloud <pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*scene_voxelised, *scene_xyz);
    scene_xyz->is_dense = false;
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_removed_scene(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*scene_xyz, *nan_removed_scene, indices);
    //ROS_INFO("NAN Removed Scene: width = %d, height = %d\n", nan_removed_scene->width, nan_removed_scene->height);

#ifdef XTION
    //offset the point cloud to ground plane for further processing
    //find minimum height
    float min_height = 99999.9999;
    for(auto p:nan_removed_scene->points)
        min_height = min(min_height,p.z);
    min_height = abs(min_height);
    for(auto &p:nan_removed_scene->points)
        p.z += min_height;
#endif

    //copy the data
    votenet_cloud = nan_removed_scene->makeShared();
    votenet_depth_image = *image;
    ros::Time end_time = ros::Time::now();
    cout<<(end_time-start_time).toSec()<<endl;
*/
}

void pubVotenetData(const ros::TimerEvent&)
{
    // transformation the pcd to world frame
//    ROS_INFO("Transformed Scene Cloud: width = %d, height = %d\n", recieved_pointcloud.width, recieved_pointcloud.height);
//    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
//    sensor_msgs::PointCloud2 scene_world;
//    tf2::doTransform(recieved_pointcloud, scene_world, transformStamped);
//    pcl::PCLPointCloud2::Ptr scene(new pcl::PCLPointCloud2());
//    pcl_conversions::toPCL(scene_world,*scene);
//    ROS_INFO("Transformed Scene Cloud: width = %d, height = %d\n", scene->width, scene->height);

    // Create the filtering object
    std::vector<float> leaf_size = {0.01f, 0.01f, 0.01f};
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr scene_voxelised(new pcl::PCLPointCloud2());
    sor.setInputCloud(scene);
    sor.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    sor.filter(*scene_voxelised);
    //ROS_INFO("Scene Voxelized: width = %d, height = %d\n", scene_voxelised->width, scene_voxelised->height);

    pcl::PointCloud <pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*scene_voxelised, *scene_xyz);
    scene_xyz->is_dense = false;
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_removed_scene(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*scene_xyz, *nan_removed_scene, indices);
    //ROS_INFO("NAN Removed Scene: width = %d, height = %d\n", nan_removed_scene->width, nan_removed_scene->height);

    pcl::PointCloud <pcl::PointXYZ>::Ptr ground_removed(new pcl::PointCloud<pcl::PointXYZ>);
    ground_removed = cropBounds(nan_removed_scene, 0,0,0,0,0.00,2.0);

    //    //publish the point cloud for votenet
    sensor_msgs::PointCloud2 pc2_msg_;
    pcl::toROSMsg(*ground_removed, pc2_msg_);
    pc2_msg_.header.frame_id = des_frame.c_str();
    pc2_msg_.header.stamp = ros::Time::now();
    my_robot::kinect_data data;
    data.scene = pc2_msg_;
    data.depth_image = recieved_depth_image;
    pub_.publish(data);
    pub_rviz.publish(pc2_msg_);
    ROS_INFO("Message Published width = %d, height = %d\n", ground_removed->width, ground_removed->height);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "kinect_subs");

#ifdef SIMULATION
    //simulation
    string point_cloud_topic = "/camera/depth/points", depth_image_topic = "/camera/depth/image_raw" ,publish_topic = "/camera/data", rviz_topic = "votenet/pcd";
#else
    //dataset
    string point_cloud_topic = "/camera/depth/points", depth_image_topic = "/camera/depth/image" ,publish_topic = "/camera/data", rviz_topic = "votenet/pcd";
#endif
    ros::NodeHandle nh;
    ros::Timer stateTimer_ = nh.createTimer(ros::Duration(4.0), pubVotenetData);
    //Topic to publish
    pub_ = nh.advertise<my_robot::kinect_data>(publish_topic.c_str(), 1);
    pub_rviz = nh.advertise<sensor_msgs::PointCloud2>(rviz_topic.c_str(), 1);
    // Subscribed Topics
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, depth_image_topic.c_str(), 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcd_sub(nh, point_cloud_topic.c_str(), 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, pcd_sub);
    sync.registerCallback(boost::bind(&myCallback, _1, _2));
    ROS_INFO("Kinect Node Time Synchroniser Set... ");

    ros::spin();
    ROS_INFO("Node Terminated.");
    return 0;
}


