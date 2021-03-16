#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import time
import math
import numpy as np
import socket
import struct
from tf.transformations import euler_from_quaternion
import tf

"""
head_tracking_interface.py
----------------------------------------------
This script is to do the following:

"""


class HeadTrackingControl():

    def __init__(self):
        # variables
        self.UDP_IP = "192.168.1.134" 
        self.UDP_PORT = 4242

        self.joystick_mode = False

        self.kp_pitch = 0.05
        self.kp_yaw = 0.05

        # instantiate the node
        rospy.init_node('head_tracking_control')

        # instantiate the services
            # udp socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.UDP_IP, self.UDP_PORT))

            # publish to the camera controller
        self.pitch_yaw_pub = rospy.Publisher('main_camera_controller/pitch_yaw', 
            Twist, queue_size=5)

            # subscribe to joystick inputs on topic "joy"
        self.joy_subscriber = rospy.Subscriber("joy", Joy, self.joystick_callback)

            # tf listener
        self.head_pose_listener = tf.TransformListener()

    def joystick_callback(self, data):
        
        if self.joystick_mode:
            pitch_yaw_cmd = Twist()

            # get pitch ad yaw motion
            pitch_yaw_cmd.angular.y = data.axes[1]
            pitch_yaw_cmd.angular.z = data.axes[0]
            
            # publish
            self.pitch_yaw_pub.publish(pitch_yaw_cmd)

    def compute_head_tracking(self):
        
        raw_data, addr = self.udp_socket.recvfrom(1024)
        head_pose_data = struct.unpack('dddddd', raw_data)
        
        # get desired pitch and yaw angles
        self.pitch_angle_des = head_pose_data[4]
        self.yaw_angle_des = head_pose_data[3]

        # get actual pitch and yaw angles
        self.head_pose_listener.waitForTransform('camera_support', 'camera_color_frame', rospy.Time(0), rospy.Duration(3))
        (_, orientation) = self.head_pose_listener.lookupTransform('camera_support', 'camera_color_frame', rospy.Time(0))
        orientation_rpy = euler_from_quaternion([orientation[0],
                                                    orientation[1], 
                                                    orientation[2], 
                                                    orientation[3]])
        angles_rpy = np.degrees(orientation_rpy)
        self.pitch_angle_actual = angles_rpy[1]
        self.yaw_angle_actual = angles_rpy[2]

    def publish_head_tracking(self):

        pitch_yaw_cmd = Twist()

        # compute command velocities

        if self.pitch_angle_des >= 10:
            pitch_yaw_cmd.angular.y = 1.5
        elif self.pitch_angle_des <= -10:
            pitch_yaw_cmd.angular.y = -1.5
        else:
            pitch_yaw_cmd.angular.y = 0
        
        if self.yaw_angle_des >= 10:
            pitch_yaw_cmd.angular.z = -1.5
        elif self.yaw_angle_des <= -10:
            pitch_yaw_cmd.angular.z = 1.5
        else:
            pitch_yaw_cmd.angular.z = 0

        # pitch_yaw_cmd.angular.y = self.kp_pitch * (self.pitch_angle_actual - self.pitch_angle_des)
        # pitch_yaw_cmd.angular.z = self.kp_yaw * (self.yaw_angle_des - self.yaw_angle_actual)

        # rospy.loginfo('The actual angles are [%f, %f], desired are [%f, %f], velocities are [%f, %f]', 
        #                                                         self.pitch_angle_actual,
        #                                                         self.yaw_angle_actual,
        #                                                         self.pitch_angle_des,
        #                                                         self.yaw_angle_des,
        #                                                         pitch_yaw_cmd.angular.y,
        #                                                         pitch_yaw_cmd.angular.z)

        # # publish
        self.pitch_yaw_pub.publish(pitch_yaw_cmd)


if __name__ == "__main__":
    try:
        camera_control = HeadTrackingControl()
        # camera_control.joystick_mode = True

        while not rospy.is_shutdown():
            camera_control.compute_head_tracking()
            camera_control.publish_head_tracking()

    except rospy.ROSInterruptException:
        pass
        
