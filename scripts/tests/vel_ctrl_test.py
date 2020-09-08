#!/usr/bin/env python
import rospy
from MAV import MAV
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import time

MASK_VELOCITY = 0b0000011111000111

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

    # while not terminou or timout:
    #     rate.sleep()
    T = 15.0
    f = 1/T
    t=0
    init_time = time.time()
    omega = 2*np.pi*f
    A = 0.6
    while not time.time() - init_time >= 20:
        t += 1/20.0
        mav.set_position_target(type_mask=MASK_VELOCITY,
                                x_velocity=0,
                                y_velocity=0,
                                z_velocity=0,
                                yaw_rate=A*np.sin(omega*t))
        mav.rate.sleep()

    """ LAND AND DISARM """
    mav.RTL()

if __name__ == "__main__":
    run()
    