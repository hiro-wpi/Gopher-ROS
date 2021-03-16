#include "ros/ros.h"
#include <string>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <ros_falcon/falconPos.h>


class SubscribeAndPublish {
private:
    ros::Subscriber falcon_pos_sub_;
    ros::Subscriber falcon_button1_sub_;
    ros::Subscriber falcon_button3_sub_;
    ros::Publisher gripper_pose_pub_;
    ros::Publisher gripper_pub_;
    std_msgs::Float64MultiArray gripper_msg;
    bool controlling_arm;

public:
    SubscribeAndPublish(ros::NodeHandle* nh, ros::NodeHandle* nh_param) {
        //Topic to publish
        falcon_pos_sub_ = nh->subscribe("falconPos", 1, &SubscribeAndPublish::process_falcon_pos, this);
        falcon_button1_sub_ = nh->subscribe("falconButton1", 1, &SubscribeAndPublish::process_falcon_button1, this);
        falcon_button3_sub_ = nh->subscribe("falconButton3", 1, &SubscribeAndPublish::process_falcon_button3, this);
        gripper_pose_pub_ = nh->advertise<geometry_msgs::Twist>("set_left_vel", 1);
        gripper_pub_ = nh->advertise<std_msgs::Float64MultiArray>("set_left_eef", 1);

        // Gripper swtich (default: open)
        gripper_msg.data = {0.0};
        controlling_arm = false;
    }
    ~SubscribeAndPublish() {}

    // This callback function executes when new falcon signal comes in
    void process_falcon_pos(const ros_falcon::falconPos pos){
        if (!controlling_arm) {
            return;
        }

        float z_map = 20;
        float xy_map = 20;

        float falcon_x = pos.X;
        float falcon_y = pos.Y;
        float falcon_z = - (pos.Z - 0.125);

        geometry_msgs::Twist twist;
        twist.linear.x = falcon_x * xy_map;
        twist.linear.y = falcon_z * z_map;
        twist.linear.z = falcon_y * xy_map;

        gripper_pose_pub_.publish(twist);
    }

    void process_falcon_button3(const std_msgs::Bool button3){
		if (gripper_msg.data[0] == 1.0){
            gripper_msg.data[0] = 0.0;
        }
        else{
            gripper_msg.data[0] = 1.0;
        }
        gripper_pub_.publish(gripper_msg);
    }

    void process_falcon_button1(const std_msgs::Bool button1){
        controlling_arm = !controlling_arm;
    }
};


int main(int argc, char *argv[]){
    ros::init(argc, argv, "falcon_interface");

    ros::NodeHandle nh(""), nh_param("~");
    SubscribeAndPublish SAPObject(&nh, &nh_param);
    
    ros::Rate r(2);
    while (true) {
        ros::spinOnce();
        r.sleep();
    }
}
