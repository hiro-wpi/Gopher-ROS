#include "ros/ros.h"

#include <string>
#include <cmath>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
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
