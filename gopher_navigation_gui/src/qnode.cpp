/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include "../include/gopher_navigation_gui/qnode.hpp"

#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_navigation_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gopher_navigation_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

  // Ros communications.
  speed_subscriber = n.subscribe<nav_msgs::Odometry>("gopher_1/base_controller/odom",1,&QNode::speedCallback,this);
  pose_subscriber = n.subscribe<nav_msgs::Odometry>("gopher_1/base_controller/odom",1,&QNode::poseCallback,this);
  laser_subscriber = n.subscribe("gopher_1/base_scan",1,&QNode::laserCallback,this);

  image_transport::ImageTransport it_(n);
  main_cam_subscriber = it_.subscribe("gopher_1/main_cam/color/image_raw", 1, &QNode::mainCamCallback, this);
  wide_main_cam_subscriber = it_.subscribe("gopher_1/main_cam_wide/color/image_raw", 1, &QNode::wideMainCamCallback, this);
  right_arm_cam_subscriber = it_.subscribe("gopher_1/right_arm_cam/color/image_raw", 1, &QNode::rightArmCamCallback, this);
  wide_right_arm_cam_subscriber = it_.subscribe("gopher_1/right_arm_cam_wide/color/image_raw", 1, &QNode::wideRightArmCamCallback, this);

  QStringListModel logging_model;

	start();
	return true;
}

void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr &msg){
  double speed_x = msg->twist.twist.linear.x;
  double speed_y = msg->twist.twist.linear.y;
  Q_EMIT speed(pow(speed_x*speed_x+speed_y*speed_y, 0.5));
}

void QNode::poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Q_EMIT position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.w);
}

void QNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg){
  float rmax = msg->range_max;
  float rmin = msg->range_min;

  double distance;
  distance = rmax;

  for (int i = 0; i < msg->ranges.size(); i++){
    float r = msg->ranges[i];
    if (r < rmax && r > rmin){
      if (r < distance){
        distance = r;
      }
    }
  }
  Q_EMIT laser_scan(distance);
}

void QNode::mainCamCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  QImage im = Mat2QImage(cv_ptr->image);
  Q_EMIT main_cam_image(im);
}

void QNode::wideMainCamCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  QImage im = Mat2QImage(cv_ptr->image);
  Q_EMIT wide_main_cam_image(im);
}

void QNode::rightArmCamCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  QImage im = Mat2QImage(cv_ptr->image);
  Q_EMIT right_arm_cam_image(im);
}

void QNode::wideRightArmCamCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  QImage im = Mat2QImage(cv_ptr->image);
  Q_EMIT wide_right_arm_cam_image(im);
}

QImage QNode::Mat2QImage(cv::Mat const& src){
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

  const float scale = 255.0;

  if (src.depth() == CV_8U) {
   if (src.channels() == 1) {
     for (int i = 0; i < src.rows; ++i) {
       for (int j = 0; j < src.cols; ++j) {
         int level = src.at<quint8>(i, j);
         dest.setPixel(j, i, qRgb(level, level, level));
       }
     }
   } else if (src.channels() == 3) {
     for (int i = 0; i < src.rows; ++i) {
       for (int j = 0; j < src.cols; ++j) {
         cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
         dest.setPixel(j, i, qRgb(bgr[0], bgr[1], bgr[2]));
       }
     }
   }
  } else if (src.depth() == CV_32F) {
   if (src.channels() == 1) {
     for (int i = 0; i < src.rows; ++i) {
       for (int j = 0; j < src.cols; ++j) {
         int level = scale * src.at<float>(i, j);
         dest.setPixel(j, i, qRgb(level, level, level));
       }
     }
   } else if (src.channels() == 3) {
     for (int i = 0; i < src.rows; ++i) {
       for (int j = 0; j < src.cols; ++j) {
         cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
         dest.setPixel(j, i, qRgb(bgr[0], bgr[1], bgr[2]));
       }
     }
   }
  }
  return dest;
}


void QNode::run() {
  ros::Rate loop_rate(60);
  std_msgs::String msg;
  std::stringstream ss;
  ss << "Node starts running";
  msg.data = ss.str();
  log(Info,msg.data);

  while ( ros::ok() ) {
		ros::spinOnce();
    loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG]" << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO]" << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO]" << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR]" << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL]" << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace gopher_navigation_gui
