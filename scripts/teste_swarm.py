#!/usr/bin/env python

from Swarm import Bee, SWARM
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import LatLon

TOL_GLOBAL = 0.00001

def go():
    rospy.init_node("head") 
    swarm = SWARM(2) 
    height = 4
    
    swarm.takeoff(height)
    swarm.rate.sleep()
    # INICIAL GLOBAL POSITION
    init_lat = []
    init_lon = []
    for mav in swarm.mavs:
        init_lat.append(mav.global_pose.latitude)
        init_lon.append(mav.global_pose.longitude)
    
    # RELATIVE POSITION
    ref_lat = []
    ref_lon = []
    for i in range(len(init_lat)):
        ref_lat.append(init_lat[0] - init_lat[i])
        ref_lon.append(init_lon[0] - init_lon[i])
    # FINAL GLOBAL POSITION
    lat = []
    lon = [] 
    for i in range(len(init_lat)):
            lat.append(init_lat[i] + ref_lat[i] + 0.00008)
            lon.append(init_lon[i] + ref_lon[i] + 0.00008)
       
    ### PARAMETRIZACAO ###
    velocity = 1

    inicial = LatLon(swarm.mavs[0].global_pose.latitude,swarm.mavs[0].global_pose.longitude) 
    final = LatLon(lat[0], lon[0]) 
    inicial_distance = inicial.distance(final) # total distance
    actual_dist = inicial_distance 

    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
    
    loop_rate = rospy.Rate(2)
    K = 1/20.0
    
    while abs(swarm.mavs[0].global_pose.latitude - lat[0]) >= TOL_GLOBAL and abs(swarm.mavs[0].global_pose.longitude - lon[0]) >= TOL_GLOBAL and not rospy.is_shutdown():
        # partial goal position
        goal_lat = []
        goal_len = []
        
        if actual_dist <= K * inicial_distance:
            v = K - (K/actual_dist)              
            p_lat = v * (lat[0] - swarm.mavs[0].global_pose.latitude)
            p_lon = v * (lon[0] - swarm.mavs[0].global_pose.longitude)
                  
        else:
            p_lat = K * (lat[0] - swarm.mavs[0].global_pose.latitude)
            p_lon = K * (lon[0] - swarm.mavs[0].global_pose.longitude)

        for i in range(len(init_lat[0])):
            goal_lat.append(init_lat[i] + ref_lat[i] + p_lat)
            goal_lon.append(init_lon[i] + ref_lon[i] + p_lon)

        inicial = LatLon(swarm.mavs[0].global_pose.latitude, swarm.mavs[0].global_pose.longitude) 
        actual_dist = inicial.distance(final)
            
        swarm.go_gps_target(goal_lat, goal_lon)
        swarm.rate.sleep()
            

        sec = rospy.get_rostime().secs 
        time = sec - init_time 

        for mav in swarm.mavs:
            rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
        
    swarm.set_altitude(1)
    rospy.logwarn("Package delivered")
    swarm.set_altitude(height)

    for i in range(100):
        rospy.logwarn("Returning to inicial position")
        swarm.set_position(0, 0, height)
        swarm.rate.sleep()

    swarm.rate.sleep()
    #swarm.RTL()
    swarm.land()
    #mav._disarm()


if __name__ == "__main__":
    go()
    
