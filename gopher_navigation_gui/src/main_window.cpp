/**
License GNU GPL
\authors chengyangkj
\details https://github.com/chengyangkj/Ros_Qt5_Gui_App
         codes for using Rviz (livrviz) in Qt5
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QPixmap>
#include <iostream>
#include "../include/gopher_navigation_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_navigation_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Auto Start and logging
  **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  if (! qnode.init()) {
    showNoMasterMessage();
  }

  /*********************
  ** Subscriber and Publisher
  **********************/
  connect(&qnode,SIGNAL(speed(double)),this,SLOT(slot_speed(double)));
  connect(&qnode,SIGNAL(position(double,double,double)),
          this,SLOT(slot_position(double,double,double)));
  connect(&qnode,SIGNAL(laser_scan(double)),this,SLOT(slot_laser_scan(double)));
  connect(&qnode,SIGNAL(main_cam_image(QImage)),this,SLOT(slot_main_cam_image(QImage)));
  connect(&qnode,SIGNAL(wide_main_cam_image(QImage)),this,SLOT(slot_wide_main_cam_image(QImage)));
  connect(&qnode,SIGNAL(right_arm_cam_image(QImage)),this,SLOT(slot_right_arm_cam_image(QImage)));
  connect(&qnode,SIGNAL(wide_right_arm_cam_image(QImage)),this,SLOT(slot_wide_right_arm_cam_image(QImage)));

  // Default interface
  interface_index = 0;
  first_or_third_flag = true;

  // Set a fake battery level
  ui.batteryLevelBar->setValue(80);
  // Hide Rviz at the beginning
  ui.rvizTab->setVisible(false);

  initRviz();
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*****************************************************************************
** Rviz
*****************************************************************************/
void MainWindow::initRviz()
{
  // TODO Implement Rviz Widget logic

  ui.rvizTree->setWindowTitle("Topics");
  ui.rvizTree->setWindowIcon(QIcon("://images/classes/Displays.svg"));

  // Global Options
  QTreeWidgetItem *Global=new QTreeWidgetItem(QStringList()<<"Global Options");
  Global->setIcon(0,QIcon(":/images/options.png"));
  ui.rvizTree->addTopLevelItem(Global);
  Global->setExpanded(true);

  QTreeWidgetItem* FixedFrame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
  Global->addChild(FixedFrame);
  // add combox
  QComboBox *frame=new QComboBox();
  frame->addItem("gopher_1/odom");
  frame->setEditable(true);
  frame->setMaximumWidth(150);
  ui.rvizTree->setItemWidget(FixedFrame,1,frame);

  QTreeWidgetItem* bcolor=new QTreeWidgetItem(QStringList()<<"Background Color");
  Global->addChild(bcolor);
  // add lineedit
  QLineEdit *colorval=new QLineEdit("48;48;48");
  colorval->setMaximumWidth(150);
  ui.rvizTree->setItemWidget(bcolor,1,colorval);

  QTreeWidgetItem* framerate=new QTreeWidgetItem(QStringList()<<"Frame Rate");
  Global->addChild(framerate);
  // add spinbox
  QSpinBox *framerateval=new QSpinBox();
  framerateval->setStyleSheet("border:none");
  framerateval->setMaximumWidth(150);
  framerateval->setRange(10,50);
  framerateval->setValue(30);
  ui.rvizTree->setItemWidget(framerate,1,framerateval);

  //Grid
  QTreeWidgetItem *Grid=new QTreeWidgetItem(QStringList()<<"Grid");
  Grid->setIcon(0,QIcon(":/images/classes/Grid.png"));
  ui.rvizTree->addTopLevelItem(Grid);
  Grid->setExpanded(true);

  QCheckBox* gridcheck=new QCheckBox;
  gridcheck->setChecked(false);
  ui.rvizTree->setItemWidget(Grid,1,gridcheck);

  QTreeWidgetItem *Grid_Status=new QTreeWidgetItem(QStringList()<<"Statue:");
  Grid_Status->setIcon(0,QIcon(":/images/ok.png"));
  Grid->addChild(Grid_Status);
  QLabel *Grid_Status_Value=new QLabel("ok");
  Grid_Status_Value->setMaximumWidth(150);
  ui.rvizTree->setItemWidget(Grid_Status,1,Grid_Status_Value);

  QTreeWidgetItem* Reference_Frame=new QTreeWidgetItem(QStringList()<<"Reference Frame");
  QComboBox* Reference_Frame_Value=new QComboBox();
  Grid->addChild(Reference_Frame);
  Reference_Frame_Value->setMaximumWidth(150);
  Reference_Frame_Value->setEditable(true);
  Reference_Frame_Value->addItem("<Fixed Frame>");
  ui.rvizTree->setItemWidget(Reference_Frame,1,Reference_Frame_Value);

  QTreeWidgetItem* Plan_Cell_Count=new QTreeWidgetItem(QStringList()<<"Plan Cell Count");
  Grid->addChild(Plan_Cell_Count);
  QSpinBox* Plan_Cell_Count_Value=new QSpinBox();
  Plan_Cell_Count_Value->setMaximumWidth(150);
  Plan_Cell_Count_Value->setRange(1,100);
  Plan_Cell_Count_Value->setValue(10);
  ui.rvizTree->setItemWidget(Plan_Cell_Count,1,Plan_Cell_Count_Value);

  QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
  QLineEdit* Grid_Color_Value=new QLineEdit();
  Grid_Color_Value->setMaximumWidth(150);
  Grid->addChild(Grid_Color);
  Grid_Color_Value->setText("160;160;160");
  ui.rvizTree->setItemWidget(Grid_Color,1,Grid_Color_Value);

  //Robot Model
  QTreeWidgetItem *Robot_Model=new QTreeWidgetItem(QStringList()<<"RobotModel");
  Robot_Model->setIcon(0,QIcon(":/images/classes/RobotModel.png"));
  ui.rvizTree->addTopLevelItem(Robot_Model);
  Robot_Model->setExpanded(true);

  QCheckBox* robotcheck=new QCheckBox;
  robotcheck->setChecked(true);
  ui.rvizTree->setItemWidget(Robot_Model,1,robotcheck);

  //Laser
  QTreeWidgetItem *Laser=new QTreeWidgetItem(QStringList()<<"LaserScan");
  Laser->setIcon(0,QIcon(":/images/classes/LaserScan.png"));
  ui.rvizTree->addTopLevelItem(Laser);
  Laser->setExpanded(true);

  QCheckBox* lasercheck=new QCheckBox;
  gridcheck->setChecked(false);
  ui.rvizTree->setItemWidget(Laser,1,lasercheck);

  QTreeWidgetItem *Laser_Status=new QTreeWidgetItem(QStringList()<<"Statue:");
  Laser_Status->setIcon(0,QIcon(":/images/ok.png"));
  Laser->addChild(Laser_Status);
  QLabel *Laser_Status_Value=new QLabel("ok");
  Laser_Status_Value->setMaximumWidth(150);
  ui.rvizTree->setItemWidget(Laser_Status,1,Laser_Status_Value);

  QTreeWidgetItem* Laser_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
  Laser->addChild(Laser_Topic);
  QComboBox *Laser_Topic_Name=new QComboBox;
  Laser_Topic_Name->setMaximumWidth(150);
  Laser_Topic_Name->addItem("gopher_1/base_scan");
  Laser_Topic_Name->setEditable(true);
  ui.rvizTree->setItemWidget(Laser_Topic,1,Laser_Topic_Name);

  //Map
  QTreeWidgetItem *Map=new QTreeWidgetItem(QStringList()<<"Map");
  Map->setIcon(0,QIcon(":/images/classes/Map.png"));
  ui.rvizTree->addTopLevelItem(Map);
  Map->setExpanded(true);

  QCheckBox* mapcheck=new QCheckBox;
  mapcheck->setChecked(false);
  ui.rvizTree->setItemWidget(Map,1,mapcheck);

  QTreeWidgetItem *Map_Status=new QTreeWidgetItem(QStringList()<<"Statue:");
  Map_Status->setIcon(0,QIcon(":/images/ok.png"));
  Map->addChild(Map_Status);
  QLabel *Map_Status_Value=new QLabel("ok");
  Map_Status_Value->setMaximumWidth(150);
  ui.rvizTree->setItemWidget(Map_Status,1,Map_Status_Value);

  QTreeWidgetItem* Map_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
  Map->addChild(Map_Topic);
  QComboBox *Map_Topic_Name=new QComboBox;
  Map_Topic_Name->setMaximumWidth(150);
  Map_Topic_Name->addItem("/map");
  Map_Topic_Name->setEditable(true);
  ui.rvizTree->setItemWidget(Map_Topic,1,Map_Topic_Name);

  QTreeWidgetItem* Map_Alpha=new QTreeWidgetItem(QStringList()<<"Alpha");
  Map->addChild(Map_Alpha);
  QLineEdit* Map_Alpha_Value=new QLineEdit("0.7");
  Map_Alpha_Value->setMaximumWidth(150);
  ui.rvizTree->setItemWidget(Map_Alpha,1,Map_Alpha_Value);

  QTreeWidgetItem* Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
  Map->addChild(Color_Scheme);
  QComboBox *Color_Scheme_Bar=new QComboBox;
  Color_Scheme_Bar->setMaximumWidth(150);
  Color_Scheme_Bar->addItems(QStringList()<<"map"<<"costmap"<<"raw");
  ui.rvizTree->setItemWidget(Color_Scheme,1,Color_Scheme_Bar);

  // Display Rviz
  map_rviz = new QRviz(ui.rvizLayout,"qrviz");

  // setup global options
  QComboBox *Global_op=static_cast<QComboBox *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(0)->child(0),1));
  QLineEdit *back_color=static_cast<QLineEdit *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(0)->child(1),1));
  QStringList coList=back_color->text().split(";");
  QColor colorBack=QColor(coList[0].toInt(),coList[1].toInt(),coList[2].toInt());
  QSpinBox *FrameRaBox=static_cast<QSpinBox *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(0)->child(2),1));
  map_rviz->SetGlobalOptions(Global_op->currentText(), colorBack, FrameRaBox->value());

  // show grid (disable by default)
  map_rviz->Display_Grid(false,Global_op->currentText(),10,QColor(160,160,160));

  // show robot model
  map_rviz->Display_RobotModel(true);

  // show laser
  QComboBox *laser_topic_box= static_cast<QComboBox *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(3)->child(1),1));
  map_rviz->Display_LaserScan(true,laser_topic_box->currentText());

  // show map
  QComboBox *map_topic_box = static_cast<QComboBox *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(4)->child(1),1));
  QLineEdit *alpha = static_cast<QLineEdit *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(4)->child(2),1));
  QComboBox *scheme = static_cast<QComboBox *> (ui.rvizTree->itemWidget(ui.rvizTree->topLevelItem(4)->child(3),1));
  map_rviz->Display_Map(true,map_topic_box->currentText(),alpha->text().toDouble(),scheme->currentText());

  // Change camera view
  //map_rviz->SetCameraView(Global_op->currentText(), 10.0, 3.14, 3.14);
}

/*****************************************************************************
** Publisher and Subscriber [Slots]
*****************************************************************************/
void MainWindow::slot_main_cam_image(QImage image){
  if (interface_index != 1 && first_or_third_flag == true){
    ui.interfaceLabel->setPixmap(QPixmap::fromImage(image).scaled(
                                      ui.interfaceLabel->width(),ui.interfaceLabel->height()));
  }
}

void MainWindow::slot_wide_main_cam_image(QImage image){
  if (interface_index == 1 && first_or_third_flag == true){
    ui.interfaceWideLabel->setPixmap(QPixmap::fromImage(image).scaled(
                                      ui.interfaceWideLabel->width(),ui.interfaceWideLabel->height()));
  }
}

void MainWindow::slot_right_arm_cam_image(QImage image){
  if (interface_index != 1 && first_or_third_flag == false){
    ui.interfaceLabel->setPixmap(QPixmap::fromImage(image).scaled(
                                      ui.interfaceLabel->width(),ui.interfaceLabel->height()));
  }
}

void MainWindow::slot_wide_right_arm_cam_image(QImage image){
  if (interface_index == 1 && first_or_third_flag == false){
    ui.interfaceWideLabel->setPixmap(QPixmap::fromImage(image).scaled(
                                      ui.interfaceWideLabel->width(),ui.interfaceWideLabel->height()));
  }
}

void MainWindow::slot_speed(double speed){
  ui.speedNumber->display(speed);
}

void MainWindow::slot_position(double x,double y,double w){
  ui.xNumber->display(x);
  ui.yNumber->display(y);
  ui.yawNumber->display(w);
}

void MainWindow::slot_laser_scan(double distance){
  double val;
  if (distance > 1.0 || distance < 0.0){
    val = 0;
  } else {
    val = 100* (1 - distance);
  }
  ui.obstacleIndicatorBar->setValue(int(val));
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_okButton_clicked()
{
  interface_index = ui.interfaceBox->currentIndex();

  ui.interfaceWideLabel->clear();
  ui.interfaceLabel->clear();
  if (interface_index == 2){
    ui.rvizTab->setVisible(true);
  }
  else{
    ui.rvizTab->setVisible(false);
  }
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
  switch (ev->key()){
  case Qt::Key_Shift:
    first_or_third_flag = !first_or_third_flag;
    break;
  }

  ui.interfaceWideLabel->clear();
  ui.interfaceLabel->clear();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),
                     tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "gopher_navigation_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "gopher_navigation_gui");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace gopher_navigation_gui

