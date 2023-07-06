

# Rospackage: gopher_navigation

## General Info
ROS Distro: ROS Noetic

## ROS Package Description

This package handles loading the navigation stack for the Gopher Robot. As of this build the package was tested with the Gopher-ROS-Unity package and simulation. 

This includes:
- running the navigation stack for the Unity simulated robot
- provides the option for using either:
    - DWA local planner (default)
    - TEB local planner
- modifications to the navigation stack for easier interfacing
    - advertised services
        - setting goal
        - canceling goal
    - subscriptions
        - footprint

## Additional Installation
Dynamic Footprint relies on the C++ Boost.Geometry library. Instalation instructions can be found on https://robots.uc3m.es/installation-guides/install-boost.html To install on Ubuntu, run:
```
sudo apt install libboost-geometry-utils-perl
```

## Running the navigation stack with Unity Simulation
    
For running the navigation stack in conjunction to the Unity simuation, run the following in seperate terminals

First Connect to the robot in Unity:

```
roslaunch gopher_endpoint gopher_endpoint.launch
```

Make sure in Unity to run the simuation, and click "Connect" in the HUD in the upper left corner of the scene view

Next launch rviz, with the DWA configuration:

```
roslaunch gopher_navigation rviz.launch local_planner:="dwa"
```

If the robot is connected at this point, laser scans and the robot transformation frames should be seen. The laser scans should be seen in red.

Launch the navigation stack

```
roslaunch gopher_navigation gopher_navigation local_planner:="dwa"
```

The map should be now loaded and seen in rviz (map is white, gray and black). Additioanlly the local_costmap should be seen (costmap is in cyan and blue).

(Optional) The change the default parameters, load the dynamic reconfigure window with:

```
rosrun rqt_reconfigure rqt_reconfigure 
```

## Modifications to the Navigation Stack

There are two goals when we were modifying the navigation stack:

- Making it easier use with Unity by converting all **actionlib servers** to **ros services** 
- Making the navigation more useful for the type of actions we intend the robot to perform

Lets go into the details:

### move_base_cancel_goal_node(std_srv/Empty)

Provides a non-action interface for canceling the goal

### move_base_send_goal_node(nav_msgs/GetPlan)

Provides a non-action interface for sending the goal. 

Note: Publishing to the move_base/goal topic was not recommended if we are using the action services of move_base.

### move_base_dynamic_footprint_node(Geometry_msgs/Polygon)

Subscriber that updates the footprint of the robot. This uses dynamic reconfigure. 