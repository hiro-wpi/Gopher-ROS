/**
License GNU GPL
\authors chengyangkj
\details https://github.com/chengyangkj/Ros_Qt5_Gui_App
         codes for using Rviz (livrviz) in Qt5
**/


#ifndef QRVIZ_H
#define QRVIZ_H

#include <QVBoxLayout>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool.h>
#include "rviz/image/ros_image_texture.h"
#include<rviz/tool_manager.h>
#include <QThread>
#include <QDebug>
#include <QException>
class QRviz:public QThread
{
    Q_OBJECT

public:
    QRviz(QVBoxLayout *layout,QString node_name);
    void run();
    void createDisplay(QString display_name,QString topic_name);
    //display Grid
    void Display_Grid(bool enable,QString Reference_frame,int Plan_Cell_count,QColor color=QColor(125,125,125));
    //display map
    void Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme);
    //global options display
    void SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate);
    //display laser
    void Display_LaserScan(bool enable,QString topic);
    //display navigation
    void Display_Navigate(bool enable,QString Global_topic,QString Global_planner,QString Local_topic,QString Local_planner);
    //display tf
    void Display_TF(bool enable);
    void Set_Pos();
    void Set_Goal();
    void Set_MoveCamera();
    void Set_Select();
    //Goal topic
    void Send_Goal_topic();
    //display robotmodel
    void Display_RobotModel(bool enable);

    void SetCameraView(QString target_frame, double distance, double pitch, double yaw);

private:
    //rvizdisplay container
    rviz::RenderPanel *render_panel_;
    rviz::VisualizationManager *manager_;
    rviz::Display* grid_=NULL ;

    //display tf
    rviz::Display* TF_=NULL ;
    rviz::Display* map_=NULL ;
    rviz::Display* laser_=NULL ;
    rviz::Display* Navigate_localmap=NULL;
    rviz::Display* Navigate_localplanner=NULL;
    rviz::Display* Navigate_globalmap=NULL;
    rviz::Display* Navigate_globalplanner=NULL;
    rviz::Display* Navigate_amcl=NULL;



    //rviz tool 
    rviz::Tool *current_tool;
    //rviz tool manager
    rviz::ToolManager *tool_manager_;
    QVBoxLayout *layout;
    QString nodename;
private slots:
    void addTool( rviz::Tool* );

//   rviz::VisualizationManager *manager_=NULL;
//    rviz::RenderPanel *render_panel_;

};

#endif // QRVIZ_H
