

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