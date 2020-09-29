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

    init_lat = []
    init_lon = []
    for mav in swarm.mavs:
        init_lat.append(mav.global_pose.latitude)
        init_lon.append(mav.global_pose.longitude)
    
    ref_lat = []
    ref_lon = []
    for i in range(len(init_lat)):
        ref_lat.append(init_lat[0] - init_lat[i])
        ref_lon.append(init_lon[0] - init_lon[i])

    lat = []
    lon = []
    for i in range(len(init_lat)):
        lat.append(init_lat[i] + ref_lat[i] + 0.00008)
        lon.append(init_lon[i] + ref_lon[i] + 0.00008)

    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))


    rospy.logwarn("SETTING GLOBAL POSITION")
    swarm.go_gps_target(lat, lon)

    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))

    swarm.set_altitude(1)
    rospy.logwarn("PACKAGE DELIVERED")
    swarm.set_altitude(height)

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
    
