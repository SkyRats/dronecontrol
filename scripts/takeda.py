#!/usr/bin/env python
import rospy
import numpy as np
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import time

def run():

    rospy.init_node("head")
    mav = MAV("1")
    height = 2
    
    """ ARM AND TAKE OFF"""
    mav.takeoff(height)
    height = 5
    R = 2
    mav.set_position(0, 0, height)

    """ LAND AND DISARM """
    mav.RTL()

if __name__ == "__main__":
    run()
    