#! /usr/bin/env python

import rospy
import roslib
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from gopher_navigation.srv import TFListenerService, TFListenerServiceRequest, TFListenerServiceResponse
from geometry_msgs.msg import TransformStamped

class TFListener():

    # Constructor
    def __init__(self):
        # Debug Message for Clarification
        rospy.loginfo("Started: tf_listener_node << Waiting for Parent and Child Frames Before Publishing")

        # For initcialization
        self.parentFrame = ""
        self.childFrame = ""
        self.isNodeInitcialized = False

    
        self.service = rospy.Service("tf_listener/set_parent_and_child", TFListenerService, self.initcializeParentAndChild)
        self.publisher = rospy.Publisher("tf_listener/transform", TransformStamped, queue_size=10)

        self.listener = tf.TransformListener()

        self.update()

    # The update loop that handles publishing the transform when ready
    def update(self):

        rate = rospy.Rate(10.0)

        while (not rospy.is_shutdown()):
            
            if(self.isNodeInitcialized):
                time = rospy.Time()

                try:
                    (position, rotation) = self.listener.lookupTransform(self.parentFrame, self.childFrame, time)

                    # convert that to a transformStamped
                    TransformStampedMsg = self.toTransfromStamped(position, rotation, time)
                
                    # publish the transformStamped msg
                    self.publisher.publish(TransformStampedMsg)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    # it may take a few attempts to connect
                    rospy.logerr(e)
                    continue

    # Checks to make sure the transformation is valid, by using the result of tf.Listener.lookupTransform
    def isValidTransform(self, parent_frame, child_frame):

        try:
            (position, rotation) =  self.listener.lookupTransform(parent_frame, child_frame, rospy.Time())
            rospy.loginfo("tf_listener_node << Valid transformation requested")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf_listener_node << Invalid transformation requested")
            return False
        
        return True


    # initcialized the string of the parent and child frame to get the correct transformation
    def initcializeParentAndChild(self, req):

        isValid = self.isValidTransform(req.parent_frame, req.child_frame)

        response = TFListenerServiceResponse()
        
        if(isValid):

            if(self.parentFrame == ""):
                rospy.loginfo("Initicailized: tf_listener_node << Frames recieved, will start publishing.")
            else:
                rospy.loginfo("tf_listener_node << New Parent and Child Frame Given")

            self.parentFrame = req.parent_frame
            self.childFrame = req.child_frame
            self.isNodeInitcialized = True

            response.isValidTransform = True

        else:
            response.isValidTransform = False

        return response

    # Converts the position, and rotation tuples to the TransformStamped msg
    def toTransfromStamped(self, position, rotation, time):
        msg = TransformStamped()

        msg.header.frame_id = self.parentFrame
        msg.header.stamp = time

        msg.child_frame_id = self.childFrame

        msg.transform.translation.x = position[0] 
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]

        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]

        return msg


if __name__ == "__main__":
    try:
        rospy.init_node("tf_listener_node")
        listener = TFListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")