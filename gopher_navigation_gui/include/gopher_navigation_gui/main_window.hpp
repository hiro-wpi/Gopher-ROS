#ifndef gopher_navigation_gui_MAIN_WINDOW_H
#define gopher_navigation_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets>
#include <QtWidgets/QMainWindow>

//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "qrviz.hpp"

//keyboard
#include <QKeyEvent>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gopher_navigation_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initRviz();

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_okButton_clicked();
  void keyPressEvent(QKeyEvent *);

  void slot_speed(double);
  void slot_position(double,double,double);
  void slot_laser_scan(double);
  void slot_main_cam_image(QImage);
  void slot_wide_main_cam_image(QImage);
  void slot_right_arm_cam_image(QImage);
  void slot_wide_right_arm_cam_image(QImage);

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  int interface_index;
  bool first_or_third_flag;

  QRviz *map_rviz=NULL;
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;
};

}  // namespace gopher_navigation_gui

#endif // gopher_navigation_gui_MAIN_WINDOW_H
