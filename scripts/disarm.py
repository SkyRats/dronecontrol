#!/usr/bin/env python

import rospy
import mavros_msgs
from mavros_msgs import srv
from geometry_msgs.msg import PoseStamped

drone_pose = PoseStamped()

def drone_disarm():
    #rospy.init_node("EmergencyDisarm")
    rospy.logwarn("DISARM MODE")
    rate = rospy.Rate(20)
    for i in range(3):
        rate.sleep()
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    if drone_pose.pose.position.z < 0.2:
        for i in range(3):
            rospy.loginfo('Drone height' + str(drone_pose.pose.position.z))
            arm(False)

if __name__ == "__main__":
    drone_disarm()
