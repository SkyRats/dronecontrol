#!/usr/bin/env python

from Swarm import Bee, SWARM
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

TOL_GLOBAL = 0.00001

def go():
    rospy.init_node("head") 
    swarm = SWARM(2) 
    height = 4
    
    swarm.takeoff(height)
    swarm.rate.sleep()

    #for i in range(100):
    #    rospy.logwarn("going off")
    #    swarm.set_position(0, 0, height)
    #    swarm.rate.sleep()

    latlon = []
    #lon =[]
    # usar um auxiliar i = 0 e somar no for
    # usar uma lista para lat e uma para lon 
    for mav in swarm.mavs:
        #latlon.append(mav.global_pose.latitude + 0.00008)
        #latlon.append(mav.global_pose.longitude + 0.00002)
        lat = mav.global_pose.latitude + 0.00008
        lon = mav.global_pose.longitude + 0.00008

        rospy.logwarn("SETTING GLOBAL POSITION")
        swarm.go_gps_target(mav,lat,lon)
        swarm.rate.sleep()
            


    swarm.set_altitude(1)
    rospy.logwarn("PACKAGE DELIVERED")
    swarm.set_altitude(height)
    
    #init_position_mavs0 = swarm.mavs0.drone_pose.pose.position
    #while abs(swarm.mav0.drone_pose.pose.position - init_position_mavs0) >= TOL_GLOBAL or abs(swarm.mavs1.drone_pose.pose.position -  >= TOL_GLOBAL:
    for i in range(100):
        rospy.logwarn("GOING BACK")
        swarm.set_position(0, 0, height)
        swarm.rate.sleep()

    swarm.rate.sleep()
    #swarm.RTL()
    swarm.land()
    #mav._disarm()


if __name__ == "__main__":
    go()


