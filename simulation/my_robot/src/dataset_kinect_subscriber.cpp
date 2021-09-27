//
// Created by vcr on 2021-01-08.
//
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
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

#define SUBSCRIBER_DELAY 8
typedef pcl::PointCloud <pcl::PointXYZ> PointCloud;
using namespace std;
bool subscribe_flag = true; // used to subsribe only once.



class KinectNode {
private:
    ros::NodeHandle n_sub, n_pub;
    ros::Publisher pub_;
    ros::Subscriber sub_;
public:
    KinectNode(string subscribe_topic, string publish_topic) {
        //Topic to publish
        pub_ = n_pub.advertise<sensor_msgs::PointCloud2>(publish_topic.c_str(), 1);
        //Topic to subscribe
        sub_ = n_sub.subscribe(subscribe_topic.c_str(), 1, &KinectNode::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &input) {

        // trsnformations
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

        try {
            transformStamped = tfBuffer.lookupTransform("world", "openni_depth_optical_frame",
                                                        ros::Time(0),
                                                        ros::Duration(3.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT transform openni_depth_optical_frame to world: %s", ex.what());
        }

        sensor_msgs::PointCloud2 scene_world;
        tf2::doTransform(*input, scene_world, transformStamped);
        cout << "Transformed Point Cloud to world ...." << endl;

        pcl::PCLPointCloud2::Ptr scene(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(scene_world,*scene);
        printf("Scene Cloud: width = %d, height = %d\n", scene->width, scene->height);

        // Create the filtering object
        std::vector<float> leaf_size = {0.01f, 0.01f, 0.01f};
        pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
        pcl::PCLPointCloud2::Ptr scene_voxelised(new pcl::PCLPointCloud2());
        sor.setInputCloud(scene);
        sor.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
        sor.filter(*scene_voxelised);
        printf("Scene Voxelised: width = %d, height = %d\n", scene_voxelised->width, scene_voxelised->height);

        pcl::PointCloud <pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*scene_voxelised, *scene_xyz);
        scene_xyz->is_dense = false;
        std::vector<int> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr nan_removed_scene(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::removeNaNFromPointCloud(*scene_xyz, *nan_removed_scene, indices);
        printf("NAN Removed Scene: width = %d, height = %d\n", nan_removed_scene->width, nan_removed_scene->height);


        //publish the point cloud for votenet
        sensor_msgs::PointCloud2 pc2_msg_;
        pcl::toROSMsg(*nan_removed_scene, pc2_msg_);
        pc2_msg_.header.frame_id = "world";
        pc2_msg_.header.stamp = ros::Time::now();
        pub_.publish(pc2_msg_);
        cout << "Published Point Cloud ......." << endl;
    }

    void simpleVis(PointCloud::Ptr cloud, std::string title) {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinect_dataset_subs");
    KinectNode kinectNode("/camera/depth/points", "/votenet/pcd");
    ros::Time prev_time = ros::Time::now();
    while (ros::ok()) {
        if (subscribe_flag) {
            ros::spinOnce();
            ros::Duration(0.01).sleep();
            subscribe_flag = false;
        }
        ros::Time curr_time = ros::Time::now();
        if ((curr_time.sec - prev_time.sec) >= SUBSCRIBER_DELAY) {
            prev_time = ros::Time::now();
            subscribe_flag = true;
        }
    }
    cout << "Node ended ...." << endl;
    return 0;
}



