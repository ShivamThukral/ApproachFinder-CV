//
// Created by vcr on 2021-04-30.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "gazebo_msgs/LinkStates.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

using namespace std;
mutex mtx;           // mutex for critical section

std::string global_frame = "odom";
std::string robot_frame = "base_footprint";

class GroundTruthPublisher
{
    ros::NodeHandle nh;
    ros::Publisher state_publisher_;  //publishes the state (Odometry_msgs)
    ros::Subscriber link_states_subscriber_;
    float hz;

//    ros::Subscriber twist_subscriber_;
    ros::Timer stateTimer_;
//    std::string src_frame, des_frame;
//    geometry_msgs::Twist curr_vel;
    nav_msgs::Odometry ground_state;
public:
    GroundTruthPublisher();
    ~GroundTruthPublisher();
    void linkStatesCallback(const gazebo_msgs::LinkStates& linkStates);
//    geometry_msgs::TransformStamped listenTransform(std::string des_frame, std::string src_frame);
    void pubState(const ros::TimerEvent&);
//    void velocityCallback(const geometry_msgs::Twist& vel);

};

GroundTruthPublisher::GroundTruthPublisher() {
    hz = 40.0;
    state_publisher_ = nh.advertise<nav_msgs::Odometry>("/ground_truth/state", 1);
    link_states_subscriber_ = nh.subscribe("/gazebo/link_states",1,&GroundTruthPublisher::linkStatesCallback, this);
    stateTimer_ = nh.createTimer(ros::Duration(1.0/hz), &GroundTruthPublisher::pubState, this);
//    twist_subscriber_ = nh.subscribe("/cmd_vel",1,&GroundTruthPublisher::velocityCallback, this);
//    src_frame = "base_footprint";
//    des_frame = "odom";
    ground_state.header.frame_id = global_frame;
    ground_state.child_frame_id = robot_frame;
    ROS_INFO("Ground Truth Publisher Setup Done.");
}

GroundTruthPublisher::~GroundTruthPublisher() {
//    stateTimer_.stop();
}

void GroundTruthPublisher::linkStatesCallback(const gazebo_msgs::LinkStates& linkStates) {
    vector<string> link_names = linkStates.name;
    int index =0;
    for(;index<link_names.size();index++)
    {
        if(link_names[index].compare("my_robot::base_footprint")==0)
        {
            mtx.lock();
            //update the frame as used by the system
            ground_state.header.stamp = ros::Time::now();
            ground_state.pose.pose  = linkStates.pose[index];
            ground_state.twist.twist = linkStates.twist[index];
            mtx.unlock();
            //cout<<ground_state.pose.pose.position.x<<"\t"<<ground_state.pose.pose.position.y<<"\t"<<ground_state.pose.pose.position.z<<"\t";
            //cout<<ground_state.twist.twist.angular.x<<"\t"<<ground_state.twist.twist.angular.y<<"\t"<<ground_state.twist.twist.angular.z<<endl;
            //state_publisher_.publish(ground_state);
            break;
        }
    }

    //ros::Duration(10).sleep(); // sleep for half a second

}
//geometry_msgs::TransformStamped GroundTruthPublisher::listenTransform(std::string des_frame, std::string src_frame) {
//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    geometry_msgs::TransformStamped transformStamped;
//    try {
//        transformStamped = tfBuffer.lookupTransform(des_frame.c_str(), src_frame.c_str(),
//                                                    ros::Time(0),
//                                                    ros::Duration(3.0));
//    } catch (tf2::TransformException &ex) {
//        ROS_WARN("Could NOT transform %s to %s: %s", des_frame.c_str(), src_frame.c_str(), ex.what());
//    }
//    return transformStamped;
//}

//void GroundTruthPublisher::velocityCallback(const geometry_msgs::Twist& vel) {
//    curr_vel = vel;
//    ROS_INFO("vel recieved");
//}

void GroundTruthPublisher::pubState(const ros::TimerEvent&) {


//    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
//    ground_state.pose.pose.position.x  = transformStamped.transform.translation.x;
//    ground_state.pose.pose.position.y  = transformStamped.transform.translation.y;
//    ground_state.pose.pose.position.z  = transformStamped.transform.translation.z;
//    ground_state.pose.pose.orientation = transformStamped.transform.rotation;
//    ground_state.twist.twist = curr_vel;
//    //update the frame as used by the system
//    ground_state.header.frame_id = "odom";
//    ground_state.child_frame_id = "base_footprint";
//    ground_state.header.stamp = ros::Time::now();
    mtx.lock();
    state_publisher_.publish(ground_state);
    mtx.unlock();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_state_pub");
    GroundTruthPublisher ground_truth_publisher;
    ros::spin();

    return 0;
}

