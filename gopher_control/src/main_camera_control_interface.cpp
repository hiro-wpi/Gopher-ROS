#include "ros/ros.h"

#include <string>
#include <cmath>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "geometry_msgs/Pose.h"

class SubscribeAndPublish {
private:
    ros::NodeHandle n_; 
    ros::Subscriber twist_sub_;
    ros::Publisher yaw_pub_;
    ros::Publisher pitch_pub_;
    ros::Publisher third_cam_pub_;

public:
    SubscribeAndPublish() {
        //Topic to publishx
        twist_sub_ = n_.subscribe("main_cam_controller/pitch_yaw", 10, &SubscribeAndPublish::process_camera_control, this);
        yaw_pub_ = n_.advertise<std_msgs::Float64>("main_cam_yaw_controller/command", 1);
        pitch_pub_ = n_.advertise<std_msgs::Float64>("main_cam_pitch_controller/command", 1);
        third_cam_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("right_arm_gen3_joint_trajectory_controller/command", 1);
    }
    ~SubscribeAndPublish() {}

    // This callback function executes when new image comes in
    void process_camera_control(const geometry_msgs::Twist twist){
        float yaw_map = 1.047;
        float pitch_map = 1.047;

        float d_yaw = twist.angular.z;
        float d_pitch = twist.angular.y;

        std_msgs::Float64 yaw_command;
        std_msgs::Float64 pitch_command;

        yaw_command.data = d_yaw * yaw_map;
        pitch_command.data = d_pitch * pitch_map;

        yaw_pub_.publish(yaw_command);
        pitch_pub_.publish(pitch_command);

        // Form trajectory controller command
        trajectory_msgs::JointTrajectory joint_goal;
        joint_goal.joint_names.resize(7);
        joint_goal.joint_names[0] = ("gopher_1/right_arm_joint_1"); 
        joint_goal.joint_names[1] = ("gopher_1/right_arm_joint_2"); 
        joint_goal.joint_names[2] = ("gopher_1/right_arm_joint_3"); 
        joint_goal.joint_names[3] = ("gopher_1/right_arm_joint_4"); 
        joint_goal.joint_names[4] = ("gopher_1/right_arm_joint_5"); 
        joint_goal.joint_names[5] = ("gopher_1/right_arm_joint_6"); 
        joint_goal.joint_names[6] = ("gopher_1/right_arm_joint_7"); 

        joint_goal.points.resize(1);
        trajectory_msgs::JointTrajectoryPoint joint_goal_points;
        joint_goal_points.positions.push_back(-2.7); 
        joint_goal_points.positions.push_back(1.5);
        joint_goal_points.positions.push_back(0.0);
        joint_goal_points.positions.push_back(1.43);
        joint_goal_points.positions.push_back(0.07 + pitch_command.data);
        joint_goal_points.positions.push_back(1.71 - yaw_command.data);
        joint_goal_points.positions.push_back(3.14);
        joint_goal.points[0] = joint_goal_points;
        joint_goal.points[0].time_from_start = ros::Duration(0.1);

        joint_goal.header.stamp = ros::Time::now();
        joint_goal.header.frame_id = "base_link";
        third_cam_pub_.publish(joint_goal);

        /*
        std_msgs::Float64MultiArray joint_goal;
        joint_goal.data.push_back(-2.8 + yaw_command.data);
        joint_goal.data.push_back(1.5);
        joint_goal.data.push_back(0.0);
        joint_goal.data.push_back(1.43);
        joint_goal.data.push_back(0.07);
        joint_goal.data.push_back(1.71 + pitch_command.data);
        joint_goal.data.push_back(3.14);
        third_cam_pub_.publish(joint_goal);
        */
    }

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_camera_interface_node");

    SubscribeAndPublish SAPObject;

    ROS_INFO("Ready to get Twist input data and send Float64 data to camera controller.");

    ros::spin();

    return 0;
}
