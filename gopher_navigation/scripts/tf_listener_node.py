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

        rospy.loginfo("Initicailized: tf_listener_node")

        # Service
        self.service = rospy.Service("tf_listener/set_parent_and_child", TFListenerService, self.setParentAndChild)
        
        # Publisher
        self.publisher = rospy.Publisher("tf_listener/transform", TransformStamped, queue_size=10)

        # Transform Listener and Dependant Variables
        self.listener = tf.TransformListener()
        self.parentFrame = "/map"
        self.childFrame = "/gopher/chassis_link"

        self.update()

    # Update Loop
    def update(self):

        rate = rospy.Rate(10.0)

        while (not rospy.is_shutdown()):
            
            # if(self.isNodeInitcialized):
            time = rospy.Time()

            try:
                (position, rotation) = self.listener.lookupTransform(self.parentFrame, self.childFrame, time)

                # convert that to a transformStamped
                TransformStampedMsg = self.toTransfromStamped(position, rotation, time)
            
                # publish the transformStamped msg
                self.publisher.publish(TransformStampedMsg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # it may take a few attempts to connect
                # rospy.logerr(e)
                continue

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

    # Checks if a transform can be made from the two frames given
    def isValidTransform(self, parentframe, childframe):

        # if loopupTransform doesnt return an error, then the transform should exist
        try:
            (position, rotation) =  self.listener.lookupTransform(parentframe, childframe, rospy.Time())
            rospy.loginfo("tf_listener_node << Valid transformation requested")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf_listener_node << Invalid transformation requested")
            return False
        
        return True


    # Changes the parent and child frame if a transform can be made
    def setParentAndChild(self, req):

        isValid = self.isValidTransform(req.parent_frame, req.child_frame)
        
        if(isValid):

            # Changes the used parents
            self.parentFrame = req.parent_frame
            self.childFrame = req.child_frame
            
            rospy.loginfo("tf_listener_node << Frames Changed Successfully.")
            return TFListenerServiceResponse(True)

        else:
            rospy.logwarn("tf_listener_node << Frames change was requested. Frames given did not give a valid transformation.")
            return TFListenerServiceResponse(False)

        # return response


if __name__ == "__main__":
    try:
        rospy.init_node("tf_listener_node")
        listener = TFListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tf_listener_node finished.")