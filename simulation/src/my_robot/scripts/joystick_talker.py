#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import sys
from geometry_msgs.msg import Twist
import dynamic_reconfigure.client

#TODO: set the linear scale and angular scale value in config file of joystick

class joy_talker:
    def __init__(self):
        #subscribe to the the cmd_vel
        rospy.Subscriber("cmd_vel_mppi",Twist, self.callback)
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.scale = 2.5
        self.client = dynamic_reconfigure.client.Client("mppi_controller", timeout=30, config_callback=self.configCallback)

    def callback(self, vel_cmd):
        #print("Given Command is {} {}".format(vel_cmd.linear.x,vel_cmd.angular.z))
        if vel_cmd.linear.x != self.linear_velocity or vel_cmd.angular.z != self.angular_velocity:
            #update the velocities
            self.linear_velocity = self.scale*vel_cmd.linear.x
            self.angular_velocity = -self.scale*vel_cmd.angular.z
            self.client.update_configuration({'user_desired_linear_speed':self.linear_velocity, 'user_desired_angular_speed':self.angular_velocity})

    def configCallback(self,config):
        rospy.loginfo("config set to {user_desired_linear_speed} {user_desired_angular_speed}".format(**config))




# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('joy_talker')
    jt = joy_talker()
    rospy.spin()