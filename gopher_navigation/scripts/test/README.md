The test folder is created for the purposes of testing the dynamic_footprint_node.

To test the dynamic_footprint_node, after connecting to the robot, in seperate terminals do:

```
roslaunch gopher_navigation gopher_navigation.launch
```
```
rostopic pub --once /move_base/footprint geometry_msgs/Polygon -f validPolygon.yaml
```