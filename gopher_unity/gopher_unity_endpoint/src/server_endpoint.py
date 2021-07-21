#!/usr/bin/env python

import rospy
from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import CompressedImage, LaserScan

# from niryo_moveit.srv import MoverService, MoverServiceRequest


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        # Ground truth
        "model_pose": RosPublisher("model_pose", Pose, queue_size=1),
        "model_vel": RosPublisher("model_vel", Twist, queue_size=1),
        # Sensor
        # camera
        "main_cam/color/image_raw/compressed": 
            RosPublisher("main_cam/color/image_raw/compressed", CompressedImage, queue_size=1),
        "right_arm_cam/color/image_raw/compressed": 
            RosPublisher("right_arm_cam/color/image_raw/compressed", CompressedImage, queue_size=1),
        "left_arm_cam/color/image_raw/compressed": 
            RosPublisher("left_arm_cam/color/image_raw/compressed", CompressedImage, queue_size=1),
        # lidar
        "base_scan": 
            RosPublisher("base_scan", LaserScan, queue_size=1)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
