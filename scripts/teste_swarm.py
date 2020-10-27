#!/usr/bin/env python

from Swarm import Bee, SWARM
import rospy 
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

TOL = 0.5
#https://stackoverflow.com/questions/1185408/converting-from-longitude-latitude-to-cartesian-coordinates CARTESIAN
# Converting lat/long to cartesian (in relation to the center of earth)
def get_cartesian(lat,lon):
    lat, lon = np.deg2rad(lat), np.deg2rad(lon)
    R = 6371 # radius of the earth
    x = R * np.cos(lat) * np.cos(lon)
    y = R * np.cos(lat) * np.sin(lon)
    #z = R *np.sin(lat)
    
    return x,y

def go():
    rospy.init_node("head") 
    swarm = SWARM(2) 
    height = 4
    
    swarm.takeoff(height)
    swarm.rate.sleep()

    init_lat = swarm.mavs[0].global_pose.latitude  # inicial latitude of the fisrt drone in the vector
    init_lon = swarm.mavs[0].global_pose.longitude  # inicial longitude of the fisrt drone in the vector
  
    lat = init_lat + 0.00008
    lon = init_lon + 0.00008
    
    ### CONVERSAO GLOBAL - CARTESIANO ###

    init_x, init_y = get_cartesian(init_lat, init_lon)
    final_x, final_y = get_cartesian(lat, lon)

    dist_x = final_x - init_x
    sinal_x = 1
    if dist_x < 0:
        sinal_x = -1

    dist_y = final_y - init_y
    sinal_y = -1
    if dist_y < 0:
        sinal_y = 1
        
    dist_tot = (dist_x**2 + dist_y**2)**(1/2)

    ### PARAMETRIZACAO ###
    
    velocity = 1
    init_time = rospy.get_rostime().secs
    
    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
        rospy.logwarn("LOCAL POSE: " + str(mav.drone_pose.pose.position))
        rospy.logwarn("GOAL POSE: x: " + str(dist_x) + " y:" + str(dist_y))
    
    actual_x = init_x
    actual_y = init_y
    actual_dist = dist_tot
    
    while actual_dist >= TOL and not rospy.is_shutdown():
        sec = rospy.get_rostime().secs 
        time = sec - init_time 
        
        # calculando o polinomio     
        p_x = ((-2 * (velocity**3) * (time**3)) / abs(dist_x)**2) + ((3*(time**2) * (velocity**2))/abs(dist_x))
        p_y = (p_x / abs(dist_x)) * abs(dist_y)
        
        swarm.set_position(sinal_x * p_x, sinal_y * p_y, swarm.mavs[0].drone_pose.pose.position.z)
        swarm.rate.sleep()

        actual_x = swarm.mavs[0].drone_pose.pose.position.x
        actual_y = swarm.mavs[0].drone_pose.pose.position.y
        actual_dist = ((dist_x-actual_x)**2 + (dist_y - actual_y)**2)**(1/2)

        #for mav in swarm.mavs:
        #    rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
        for mav in swarm.mavs:
            #rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
            rospy.logwarn("LOCAL POSE: " + str(mav.drone_pose.pose.position))
            rospy.logwarn("GOAL POSE: x: " + str(dist_x) + " y:" + str(dist_y))
        
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


 
