#!/usr/bin/env python

import rospy
from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import CompressedImage, LaserScan, JointState

# from niryo_moveit.srv import MoverService, MoverServiceRequest


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        # Ground truth
        "model_pose": RosPublisher("model_pose", PoseStamped, queue_size=1),
        "model_twist": RosPublisher("model_twist", TwistStamped, queue_size=1),
        
        # Sensors
        # camera
        "main_cam/color/image_raw/compressed": 
            RosPublisher("main_cam/color/image_raw/compressed", CompressedImage, queue_size=1),
        "right_arm_cam/color/image_raw/compressed": 
            RosPublisher("right_arm_cam/color/image_raw/compressed", CompressedImage, queue_size=1),
        "left_arm_cam/color/image_raw/compressed": 
            RosPublisher("left_arm_cam/color/image_raw/compressed", CompressedImage, queue_size=1),
        # lidar
        "base_scan": 
            RosPublisher("base_scan", LaserScan, queue_size=1),
        
        # State
        "joint_states": 
            RosPublisher("joint_states", JointState, queue_size=1),
        
        # Controllers
        # base controller
        "base_controller/cmd_vel":
            RosSubscriber("base_controller/cmd_vel", Twist, tcp_server),
        # camera controller
        "main_cam_pitch_controller/command":
            RosSubscriber("main_cam_pitch_controller/command", Float64, tcp_server),
        "main_cam_yaw_controller/command":
            RosSubscriber("main_cam_yaw_controller/command", Float64, tcp_server),
        # joint controllers
        "joint_1_position_controller/command":
            RosSubscriber("right_arm_joint_1_position_controller/command", Float64, tcp_server),
        "joint_2_position_controller/command":
            RosSubscriber("right_arm_joint_2_position_controller/command", Float64, tcp_server),
        "joint_3_position_controller/command":
            RosSubscriber("right_arm_joint_3_position_controller/command", Float64, tcp_server),
        "joint_4_position_controller/command":
            RosSubscriber("right_arm_joint_4_position_controller/command", Float64, tcp_server),
        "joint_5_position_controller/command":
            RosSubscriber("right_arm_joint_5_position_controller/command", Float64, tcp_server),
        "joint_6_position_controller/command":
            RosSubscriber("right_arm_joint_6_position_controller/command", Float64, tcp_server),
        "joint_7_position_controller/command":
            RosSubscriber("right_arm_joint_7_position_controller/command", Float64, tcp_server),
        "joint_1_position_controller/command":
            RosSubscriber("left_arm_joint_1_position_controller/command", Float64, tcp_server),
        "joint_2_position_controller/command":
            RosSubscriber("left_arm_joint_2_position_controller/command", Float64, tcp_server),
        "joint_3_position_controller/command":
            RosSubscriber("left_arm_joint_3_position_controller/command", Float64, tcp_server),
        "joint_4_position_controller/command":
            RosSubscriber("left_arm_joint_4_position_controller/command", Float64, tcp_server),
        "joint_5_position_controller/command":
            RosSubscriber("left_arm_joint_5_position_controller/command", Float64, tcp_server),
        "joint_6_position_controller/command":
            RosSubscriber("left_arm_joint_6_position_controller/command", Float64, tcp_server),
        "joint_7_position_controller/command":
            RosSubscriber("left_arm_joint_7_position_controller/command", Float64, tcp_server)

        # 'niryo_moveit': RosService('niryo_moveit', MoverService)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
