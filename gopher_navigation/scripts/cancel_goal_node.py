#! /usr/bin/env python


import rospy
import roslib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse

def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.cancel_all_goals()

def cancel_goal(req):
    movebase_client()
    rospy.loginfo("Goal Execution Done")
    return EmptyResponse()
    
if __name__ == "__main__":
    rospy.loginfo("Start Node")
    try:
        rospy.init_node("move_base_cancel_goal_node")
        s = rospy.Service("move_base_cancel_goal", Empty, cancel_goal)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



    