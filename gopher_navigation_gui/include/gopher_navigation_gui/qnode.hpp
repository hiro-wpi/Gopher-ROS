/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gopher_navigation_gui_QNODE_HPP_
#define gopher_navigation_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <string>

#include <QThread>
#include <QStringListModel>
#include <QLabel>
#include <QImage>
#include <QSettings>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_navigation_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void main_cam_image(QImage);
  void wide_main_cam_image(QImage);
  void right_arm_cam_image(QImage);
  void wide_right_arm_cam_image(QImage);
  void speed(double);
  void position(double,double,double);
  void laser_scan(double);

private:
	int init_argc;
  char** init_argv;

  ros::Subscriber pose_subscriber;
  ros::Subscriber speed_subscriber;
  ros::Subscriber laser_subscriber;
  image_transport::Subscriber main_cam_subscriber;
  image_transport::Subscriber wide_main_cam_subscriber;
  image_transport::Subscriber right_arm_cam_subscriber;
  image_transport::Subscriber wide_right_arm_cam_subscriber;

  QStringListModel logging_model;

  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void mainCamCallback(const sensor_msgs::ImageConstPtr& msg);
  void wideMainCamCallback(const sensor_msgs::ImageConstPtr& msg);
  void rightArmCamCallback(const sensor_msgs::ImageConstPtr& msg);
  void wideRightArmCamCallback(const sensor_msgs::ImageConstPtr& msg);
  QImage Mat2QImage(cv::Mat const& src);
};

}  // namespace gopher_navigation_gui

#endif /* gopher_navigation_gui_QNODE_HPP_ */
