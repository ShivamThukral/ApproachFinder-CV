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

#define SUBSCRIBER_DELAY 10
typedef pcl::PointCloud <pcl::PointXYZ> PointCloud;
using namespace std;
bool subscribe_flag = true; // used to subsribe only once.

// https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
// can use to class stucture if evrythign works fine
//void callback(const sensor_msgs::PointCloud2ConstPtr& input)
//{
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::fromROSMsg (*input, cloud);
//    printf ("Cloud: width = %d, height = %d\n", cloud.width, cloud.height);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
//    //simpleVis(cloud_ptr,"view");
//
//    // trsnformations
//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    geometry_msgs::TransformStamped transformStamped;
//
//    try{
//        transformStamped = tfBuffer.lookupTransform("base_footprint", "camera_depth_optical_frame", ros::Time::now(),
//                                                    ros::Duration(3.0));
//    } catch (tf2::TransformException &ex) {
//        ROS_WARN("Could NOT transform camera_depth_optical_frame to link_chassis: %s", ex.what());
//    }
//    //cout<<"Transform from \"camera_depth_optical_frame\" to \"base_footprint\""<<endl;
//    //cout<<transformStamped<<endl;
//    sensor_msgs::PointCloud2 cloud_out;
//    tf2::doTransform(*input, cloud_out, transformStamped);
//
//    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
//    pcl::fromROSMsg (cloud_out, cloud_transformed);
//    cout<<"Transforming pcd to world frame ... "<<endl;
//    printf ("Cloud: width = %d, height = %d\n", cloud_transformed.width, cloud_transformed.height);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud_transformed));
//    //simpleVis(cloud_transformed_ptr,"view");
//    cout<<"PCD transformed to world frame"<<endl;
//
//    cout<<"Removing Nan's form the PCD...."<<endl;
//    cloud_transformed_ptr->is_dense = false;
//    std::vector<int> indices;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::removeNaNFromPointCloud(*cloud_transformed_ptr, *nan_removed_cloud, indices);
//    printf ("Cloud: width = %d, height = %d\n", nan_removed_cloud->width, nan_removed_cloud->height);
//
//    //publish the point cloud for votenet
//    sensor_msgs::PointCloud2 pc2_msg_;
//    pcl::toROSMsg(*nan_removed_cloud, pc2_msg_);
//    pc2_msg_.header.frame_id = "base_footprint";
//    pc2_msg_.header.stamp = ros::Time::now();
//    pub.publish(pc2_msg_);
//    cout<<"Point Cloud 2 published...."<<endl;
//    /*
//    cout<<"Writing pcd ....."<<endl;
//    pcl::PCLPointCloud2 ply_cloud;
//    pcl::toPCLPointCloud2(*nan_removed_cloud, ply_cloud);
//    string filename = "/home/vcr/Desktop/kinect_world_final.ply";
//    pcl::PLYWriter writer;
//    bool binary = true,use_camera=true;
//    writer.write(filename, ply_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),binary,use_camera);
//     */
//
//}

class KinectNode {
private:
    ros::NodeHandle n_sub, n_pub;
    ros::Publisher pub_;
    ros::Subscriber sub_;
public:
    KinectNode(string subscribe_topic, string publish_topic) {
        //Topic to publish
        pub_ = n_pub.advertise<sensor_msgs::PointCloud2>("/votenet/pcd", 1);
        //Topic to subscribe
        sub_ = n_sub.subscribe("/camera/depth/points", 1, &KinectNode::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &input) {

        // trsnformations
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

        try {
            transformStamped = tfBuffer.lookupTransform("odom", "camera_depth_optical_frame",
                                                        ros::Time(0),
                                                        ros::Duration(3.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT transform camera_depth_optical_frame to odom: %s", ex.what());
        }

        sensor_msgs::PointCloud2 scene_world;
        tf2::doTransform(*input, scene_world, transformStamped);
        cout << "Transformed Point Cloud to odom ...." << endl;

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
        pc2_msg_.header.frame_id = "odom";
        pc2_msg_.header.stamp = ros::Time::now();
        pub_.publish(pc2_msg_);
        cout << "Published Point Cloud ...." << endl;
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
    ros::init(argc, argv, "kinect_subs");
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


