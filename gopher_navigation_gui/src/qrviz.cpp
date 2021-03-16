/**
License GNU GPL
\authors chengyangkj
\details https://github.com/chengyangkj/Ros_Qt5_Gui_App
         codes for using Rviz (livrviz) in Qt5
**/


#include "../include/gopher_navigation_gui/qrviz.hpp"

QRviz::QRviz(QVBoxLayout *layout,QString node_name)
{

    this->layout=layout;
    this->nodename=node_name;


    //create rviz container
    render_panel_=new rviz::RenderPanel;
    //add widget to layout
    layout->addWidget(render_panel_);
    //initilize rviz object
    manager_=new rviz::VisualizationManager(render_panel_);
    ROS_ASSERT(manager_!=NULL);
    //acquire tool object
    tool_manager_=manager_->getToolManager();
    ROS_ASSERT(tool_manager_!=NULL);
    //Initialize camera
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    tool_manager_->initialize();
    manager_->removeAllDisplays();
    // Initialize camera type and view
    rviz::ViewManager* vm_ = manager_->getViewManager();
    vm_->setCurrentViewControllerType("rviz/ThirdPersonFollower");
    vm_->getCurrent()->subProp("Target Frame")->setValue("gopher_1/odom");
    vm_->getCurrent()->subProp("Distance")->setValue("20");
    vm_->getCurrent()->subProp("Pitch")->setValue("1.57");
    vm_->getCurrent()->subProp("Yaw")->setValue("3.14");
}

rviz::Display* RobotModel_=NULL;
//display robotModel
void QRviz::Display_RobotModel(bool enable)
{

    if(RobotModel_==NULL)
    {
        RobotModel_ = manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",enable);
    }
    else{
        delete RobotModel_;
        RobotModel_ = manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",enable);
    }
  }

//display grid
void QRviz::Display_Grid(bool enable,QString Reference_frame,int Plan_Cell_count,QColor color)
{
    if(grid_==NULL)
    {
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);
        grid_->subProp( "Reference Frame" )->setValue(Reference_frame);
        grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);

    }
    else{
        delete grid_;
        grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( grid_ != NULL );
        // Configure the GridDisplay the way we like it.
        grid_->subProp( "Line Style" )->setValue("Billboards");
        grid_->subProp( "Color" )->setValue(color);
        grid_->subProp( "Reference Frame" )->setValue(Reference_frame);
        grid_->subProp("Plane Cell Count")->setValue(Plan_Cell_count);
    }
    grid_->setEnabled(enable);
    manager_->startUpdate();
}

//display map
void QRviz::Display_Map(bool enable,QString topic,double Alpha,QString Color_Scheme)
{
    if(!enable&&map_)
    {
        map_->setEnabled(false);
        return ;
    }
    if(map_==NULL)
    {
        map_=manager_->createDisplay("rviz/Map","QMap",true);
        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);

    }
    else{
         ROS_ASSERT(map_);
         qDebug()<<"asdasdasd:"<<topic<<Alpha;

        delete map_;
        map_=manager_->createDisplay("rviz/Map","QMap",true);
        ROS_ASSERT(map_);
        map_->subProp("Topic")->setValue(topic);
        map_->subProp("Alpha")->setValue(Alpha);
        map_->subProp("Color Scheme")->setValue(Color_Scheme);
    }

    map_->setEnabled(enable);
    manager_->startUpdate();
}

//display laser
void QRviz::Display_LaserScan(bool enable,QString topic)
{
    if(laser_==NULL)
    {
        laser_=manager_->createDisplay("rviz/LaserScan","QLaser",enable);
        ROS_ASSERT(laser_);
        laser_->subProp("Topic")->setValue(topic);
    }
    else{
        delete laser_;
        laser_=manager_->createDisplay("rviz/LaserScan","QLaser",enable);
        ROS_ASSERT(laser_);
        laser_->subProp("Topic")->setValue(topic);
    }
    qDebug()<<"topic:"<<topic;
    laser_->setEnabled(enable);
    manager_->startUpdate();
}

//Global options
void QRviz::SetGlobalOptions(QString frame_name,QColor backColor,int frame_rate)
{
    manager_->setFixedFrame(frame_name);
    manager_->setProperty("Background Color",backColor);
    manager_->setProperty("Frame Rate",frame_rate);
    manager_->startUpdate();
}

//set initial pos
void QRviz::Set_Pos()
{
    //acquire tool
    current_tool= tool_manager_->addTool("rviz/SetInitialPose");
    //set current tool
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();
//    tool_manager_->setCurrentTool()
}

//Set robot goal
void QRviz::Set_Goal()
{
    //tool manager
    current_tool= tool_manager_->addTool("rviz/SetGoal");
    //set goal topic
    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    //set current frame
    manager_->setFixedFrame("/map");
    //set current tool
    tool_manager_->setCurrentTool( current_tool );

    manager_->startUpdate();
}

void QRviz::Set_MoveCamera()
{
    //acquire tool
    current_tool= tool_manager_->addTool("rviz/MoveCamera");
    //set current tool
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();
}

void QRviz::Set_Select()
{
    //acquire tool
    current_tool= tool_manager_->addTool("rviz/Select");
    //set current tool
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();
}

//display tf
void QRviz::Display_TF(bool enable)
{
    if(TF_){delete TF_;TF_=NULL;}
    TF_=manager_->createDisplay("rviz/TF","QTF",enable);
}

//display navigation
void QRviz::Display_Navigate(bool enable,QString Global_topic,QString Global_planner,QString Local_topic,QString Local_planner)
{
    if(Navigate_localmap) {delete Navigate_localmap; Navigate_localmap=NULL;}
    if(Navigate_localplanner) {delete Navigate_localplanner; Navigate_localplanner=NULL;}
    if(Navigate_globalmap) {delete Navigate_globalmap; Navigate_globalmap=NULL;}
    if(Navigate_globalplanner) {delete Navigate_globalplanner; Navigate_globalplanner=NULL;}
    //local map
    Navigate_localmap=manager_->createDisplay("rviz/Map","Qlocalmap",enable);
    Navigate_localmap->subProp("Topic")->setValue(Local_topic);
    Navigate_localmap->subProp("Color Scheme")->setValue("costmap");
    Navigate_localplanner=manager_->createDisplay("rviz/Path","QlocalPath",enable);
    Navigate_localplanner->subProp("Topic")->setValue(Local_planner);
    Navigate_localplanner->subProp("Color")->setValue(QColor(0,12,255));
    //global map
    Navigate_globalmap=manager_->createDisplay("rviz/Map","QGlobalmap",enable);
    Navigate_globalmap->subProp("Topic")->setValue(Global_topic);
    Navigate_globalmap->subProp("Color Scheme")->setValue("costmap");
    Navigate_globalplanner=manager_->createDisplay("rviz/Path","QGlobalpath",enable);
    Navigate_globalplanner->subProp("Topic")->setValue(Global_planner);
    Navigate_globalplanner->subProp("Color")->setValue(QColor(255,0,0));
    //update display 
    manager_->startUpdate();
}

void QRviz::SetCameraView(QString target_frame, double distance, double pitch, double yaw){
    rviz::ViewManager* vm_ = manager_->getViewManager();
    vm_->getCurrent()->subProp("Target Frame")->setValue(target_frame);
    vm_->getCurrent()->subProp("Distance")->setValue(distance);
    vm_->getCurrent()->subProp("Pitch")->setValue(pitch);
    vm_->getCurrent()->subProp("Yaw")->setValue(yaw);
}

void QRviz::addTool( rviz::Tool* )
{
}
void QRviz::createDisplay(QString display_name,QString topic_name)
{
}
void QRviz::run()
{
}
