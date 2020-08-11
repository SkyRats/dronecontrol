#!/usr/bin/env python
import rospy
import numpy as np
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import time

def run():

    rospy.init_node("head")
    h_detect_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)

    cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
    mav = MAV("1")
    height = 2
    
    """ ARM AND TAKE OFF"""
    mav.takeoff(height)
    height = 5
    R = 2
    mav.set_position(0, R, height)

    T = 15.0
    f = 1/T
    t=0
    init_time = time.time()
    omega = 2*np.pi*f
    while not time.time() - init_time >= 2*np.pi*T and not rospy.is_shutdown():
        rospy.loginfo("({},{},{})".format(R*np.sin(omega*t), R*np.cos(omega*t), height))
        mav.set_position(R*np.sin(omega*t), R*np.cos(omega*t), height)
        t += 1/20.0
        mav.rate.sleep()

    """ LAND AND DISARM """
    mav.RTL()

if __name__ == "__main__":
    run()
    