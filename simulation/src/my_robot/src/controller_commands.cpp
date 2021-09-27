//
// Created by vcr on 2021-04-30.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <autorally_msgs/chassisCommand.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include <boost/thread/mutex.hpp>

using namespace std;

#define WHEEL_BASE 0.472   //copied from the gazebo file
#define pi  3.14159265359
float rear_axle_max_effort = 0.9;   //scale as joystick turbo enable
float steering_scale = 0.9;
int enable_index = 5;       //Right upper button
ros::Time lastPublishTime;
boost::mutex access_guard_;
geometry_msgs::TransformStamped transformStamped;

class AckermannControlCommands{
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_controls_, subscriber_joystick_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist vel_twist_msg_;
    std::string src_frame_, des_frame_;
    bool joystick_published_;
    ros::Timer tfTimer_;
public:
    AckermannControlCommands();
    //void controlCallback(const autorally_msgs::chassisCommand::ConstPtr& control);
    void customControllerCallback(const autorally_msgs::chassisCommand::ConstPtr& control);
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& command);
    void fetchTransform(const ros::TimerEvent&);
    geometry_msgs::TransformStamped listenTransform(std::string des_frame, std::string src_frame);
};

AckermannControlCommands::AckermannControlCommands() {
    //subscriber_controls_ = nh_.subscribe("/mppi_controller/chassisCommand",1,&AckermannControlCommands::controlCallback, this);
    subscriber_controls_ = nh_.subscribe("/mppi_controller/chassisCommand",1,&AckermannControlCommands::customControllerCallback, this);
    subscriber_joystick_ = nh_.subscribe("/joy",1,&AckermannControlCommands::joystickCallback, this);
    tfTimer_ = nh_.createTimer(ros::Duration(0.333), &AckermannControlCommands::fetchTransform, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_autorally", 1);
    src_frame_ = "base_footprint";
    des_frame_ = "odom";
    joystick_published_ = false;
    ROS_INFO("Controller setup done!");
}

geometry_msgs::TransformStamped AckermannControlCommands::listenTransform(std::string des_frame, std::string src_frame) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(des_frame_.c_str(), src_frame_.c_str(),
                                                    ros::Time(0),
                                                    ros::Duration(1,0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform %s to %s: %s", des_frame.c_str(), src_frame.c_str(), ex.what());
    }
    return transformStamped;
}

void AckermannControlCommands::fetchTransform(const ros::TimerEvent&) {
    //transformStamped = listenTransform(des_frame_.c_str(), src_frame_.c_str());
    //publish a zero command if we haven't recieved anything for 2 seconds
    if( ros::Time::now().toSec() - lastPublishTime.toSec() > ros::Duration(1.0).toSec())
    {
        //ROS_WARN("No Command recieved for 2 seconds - stopping robot....");
        vel_twist_msg_.linear.x = 0;
        vel_twist_msg_.linear.y = 0;
        vel_twist_msg_.angular.z = 0;
        cmd_vel_pub_.publish(vel_twist_msg_);
    }

}

//void AckermannControlCommands::controlCallback(const autorally_msgs::chassisCommand::ConstPtr& control) {
//    ROS_INFO("Control Recieved!!");
//    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
//    float q0 = transformStamped.transform.rotation.x;
//    float q1 = transformStamped.transform.rotation.y;
//    float q2 = transformStamped.transform.rotation.z;
//    float q3 = transformStamped.transform.rotation.w;
//
//    float steer_ang = -1.0*control->steering;
//    float speed = -rear_axle_max_effort*control->throttle;
//
//    //Update euler angles. These use the 1-2-3 Euler angle convention.
//    float roll = atan2(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
//    float pitch = -asin(2*q1*q3 - 2*q0*q2);
//    float yaw = atan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
//
//    //update the twist msgs
//    vel_twist_msg_.linear.x = speed*cos(yaw);
//    vel_twist_msg_.linear.y = speed*sin(yaw);
//    vel_twist_msg_.angular.z = speed * tan(steer_ang) / WHEEL_BASE;
//    cmd_vel_pub_.publish(vel_twist_msg_);
//}

void AckermannControlCommands::joystickCallback(const sensor_msgs::Joy::ConstPtr& command)
{
    boost::mutex::scoped_lock lock(access_guard_);
    if(command->buttons[enable_index] == 1)
        joystick_published_ = true;
    else
        joystick_published_=  false;
}

void AckermannControlCommands::customControllerCallback(const autorally_msgs::chassisCommand::ConstPtr& control)
{
    boost::mutex::scoped_lock lock(access_guard_);
    string sender = control->sender;

    if(abs(control->steering) <= 1.0 && abs(control->throttle) <= 1.0 && (ros::Time::now()-control->header.stamp).toSec() < ros::Duration(0.2).toSec())
    {
        float steer_ang = steering_scale*control->steering;
        float speed = rear_axle_max_effort*control->throttle;


        float q0 = transformStamped.transform.rotation.x;
        float q1 = transformStamped.transform.rotation.y;
        float q2 = transformStamped.transform.rotation.z;
        float q3 = transformStamped.transform.rotation.w;

        //Update euler angles. These use the 1-2-3 Euler angle convention.
        float roll = atan2(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
        float pitch = -asin(2*q1*q3 - 2*q0*q2);
        float yaw = atan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);

        //update the twist msgs
        if(joystick_published_)
        {
//            vel_twist_msg_.linear.x = speed*cos(yaw);
//            vel_twist_msg_.linear.y = speed*sin(yaw);
            vel_twist_msg_.linear.x = speed;
            vel_twist_msg_.angular.z = speed * tan(steer_ang) / WHEEL_BASE;
            cmd_vel_pub_.publish(vel_twist_msg_);
            lastPublishTime = control->header.stamp;
        }
//        else
//        {
//            vel_twist_msg_.linear.x = 0;
//            vel_twist_msg_.linear.y = 0;
//            vel_twist_msg_.angular.z = 0;
//        }

        // ROS_INFO("in control of steering");
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_commands");
    //create the object of ackermann steering class
    AckermannControlCommands ackermannControlCommands;
    ros::spin();
    return 0;
}