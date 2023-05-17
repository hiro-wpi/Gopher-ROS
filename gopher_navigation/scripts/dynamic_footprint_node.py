#! /usr/bin/env python


import rospy
import roslib
import dynamic_reconfigure.client
from geometry_msgs.msg import Polygon

# set the dynamic footprint parameter 
def setFootprint(pointsList):
    rospy.loginfo("dynamic_footprint_node: Setting dynamic footprint paramters to %s", str(pointsList))
    # Sets the local costmap
    local_client = dynamic_reconfigure.client.Client("move_base/local_costmap")
    params = { "footprint" : pointsList}
    config = local_client.update_configuration(params)

    # Sets the global costmap
    global_client = dynamic_reconfigure.client.Client("move_base/global_costmap")
    params = { "footprint" : pointsList}
    config = global_client.update_configuration(params)

# change the polygon msg to a list
def polygonToList(poly):
    
    pointsList = []

    for point in poly.points:
        pointsList.append([point.x, point.y])

    return pointsList


# check the Polygon recieved, and see if it is valid
#   it need 3 or more points to be a valid polygon
def isValidPolygon(poly):
    
    if(len(poly.points) < 3 ):
        rospy.logwarn("dynamic_footprint_node: Input ignored. Given list has less than 3 points. Consider adding more points.")
        return False
    
    return True


# sets the footprint of the robot given a valid polygon, ignores it otherwise
def change_footprint(req):

    if(isValidPolygon(req) == True):
        pointsList = polygonToList(req)
        setFootprint(pointsList)


if __name__ == "__main__":
    
    try:
        rospy.init_node("dynamic_footprint_node")
        rospy.loginfo("Initcialized: dynamic_footprint_node")
        rospy.Subscriber("move_base/footprint", Polygon, change_footprint)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("dynamic_footprint_node died")



    