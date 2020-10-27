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

    init_lat = [] #latitude inicial
    init_lon = [] #longitude inicial
    for mav in swarm.mavs:
        init_lat.append(mav.global_pose.latitude)
        init_lon.append(mav.global_pose.longitude)
    
    ref_lat = []
    ref_lon = []
    for i in range(len(init_lat)):
        ref_lat.append(init_lat[0] - init_lat[i])
        ref_lon.append(init_lon[0] - init_lon[i])

    lat = [] #latitude dada
    lon = [] # longitude dada
    for i in range(len(init_lat)):
        lat.append(init_lat[i] + ref_lat[i] + 0.00008)
        lon.append(init_lon[i] + ref_lon[i] + 0.00008)

    ### PARAMETRIZAÇÃO ###
    c = 100000
    dist_lat = abs(lat[0] - init_lat[0]) * c
    sinal_lat = 1 # distancia eh positiva
    if lat[0] < init_lat[0]:
        sinal_lat = -1 # distancia eh negativa

    dist_lon = abs(lon[0] - init_lon[0]) * c
    sinal_lon = 1 # distancia eh positiva
    if lat[0] < init_lat[0]:
        sinal_lon = -1 # distancia eh negativa
    
    # parametros do polinomio
    velocity = 1
    init_time = rospy.get_rostime().secs
    init_p_lat = 0
    init_p_lon = 0
    
    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))

    while abs(swarm.mav[0].global_pose.latitude - lat[0]) >= TOL and not rospy.is_shutdown():
        sec = rospy.get_rostime().secs 
        time = sec - init_time 

        # calculando o polinomio     
        p_lat = ((-2 * (velocity**3) * (time**3)) / dist_lat**2) + ((3*(time**2) * (velocity**2))/dist_lat)
        p_lon = (p_lat / dist_lat) * dist_lon
        

        set_lat = []
        set_lon = []
        for i in range(len(init_lat)):
            if init_p_lat < p_lat:
                set_lat.append(init_lat[i] + ref_lat[i] + sinal_lat * p_lat / c)
                init_p_lat = p_lat
            else:
                set_lat.append(init_lat[i] + ref_lat[i] + sinal_lat * init_p_lat / c)

            if init_p_lon < p_lon:
                set_lon.append(init_lon[i] + ref_lon[i] + sinal_lon * p_lon / c)
                init_p_lon = p_lon
            else:
                set_lon.append(init_lon[i] + ref_lon[i] + sinal_lon * init_p_lon / c)     

        swarm.go_gps_target(set_lat, set_lon)
        swarm.rate.sleep()

        for mav in swarm.mavs:
            rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
        
    '''
    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))

    rospy.logwarn("SETTING GLOBAL POSITION")
    swarm.go_gps_target(lat, lon)

    for mav in swarm.mavs:
        rospy.logwarn("GLOBAL POSE: " + str(mav.global_pose))
    '''
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
    