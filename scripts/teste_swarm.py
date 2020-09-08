#!/usr/bin/env python

from Swarm import Bee, SWARM
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

def go():
    rospy.init_node("head") 
    swarm = SWARM(2) #é assim  mesmo? esse chat ;p
    height = 4
    
    swarm.takeoff(height)
    swarm.rate.sleep()
    swarm.set_position(0, 0, height)
    swarm.rate.sleep()
    
    for x in range (150):
        rospy.logwarn("SETTING POSITION")
        swarm.set_position(3, 3, height)
        swarm.rate.sleep()
    
    swarm.set_altitude(1)
    rospy.logwarn("PEGUE A SUA ENCOMENDA")
    swarm.set_altitude(height)

    for y in range (100):
        rospy.logwarn("GOING BACK")
        swarm.set_position(0, 0, height)
        swarm.rate.sleep()

    swarm.rate.sleep()
    swarm.RTL()
    #mav.land()
    #mav._disarm()


if __name__ == "__main__":
    go()


