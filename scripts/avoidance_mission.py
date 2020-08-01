import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from MAV import MAV

laser_data = LaserScan()

def laser_callback(data):
    global laser_data
    laser_data = data

def run():
    rospy.init_node("avoidance")
    laser_sub = rospy.Subscriber("/laser/scan", LaserScan, laser_callback, queue_size=1)
    mav = MAV("1")
    goal = np.array([8, 0])
    #mav.takeoff(1.5)
    Kr = 0.5 # repulsive
    Ka = 0.3 #attractive

    d = (mav.drone_pose.pose.position.x - goal[0])**2 + (mav.drone_pose.pose.position.y - goal[1])**2
    d = np.sqrt(d)
    if mav.drone_pose.pose.orientation.z == 0:
        theta_goal = 0.0
    else:
        theta_goal = np.tan((mav.drone_pose.pose.position.x - goal[0])/(mav.drone_pose.pose.position.y - goal[1]))
    while not rospy.is_shutdown():
        d = (mav.drone_pose.pose.position.x - goal[0])**2 + (mav.drone_pose.pose.position.y - goal[1])**2
        d = np.sqrt(d)
        if mav.drone_pose.pose.orientation.z == 0:
            theta_goal = 0.0
        else:
            theta_goal = np.tan((mav.drone_pose.pose.position.x - goal[0])/(mav.drone_pose.pose.position.y - goal[1]))
        theta = laser_data.angle_min
        Ft = np.array([0.0, 0.0])
        Fg = np.array([Ka*d*np.cos(theta_goal - mav.drone_pose.pose.orientation.z),
                                    Ka*d*np.sin(theta_goal- mav.drone_pose.pose.orientation.z)])
        #rospy.loginfo(laser_data)
        for laser_range in laser_data.ranges:
            theta += laser_data.angle_increment
            if laser_range != "inf": # not sure if that's how we verify
                Fi = Kr/(laser_range**3)
                Fix = -Fi*np.cos(theta)
                Fiy = -Fi*np.sin(theta)
                Ft += np.array([Fix, Fiy])

        F = Ft + Fg
        #rospy.loginfo("Force = {}".format(F))
        if mav.drone_state != "OFFBOARD":
            rospy.loginfo("SETTING OFFBOARD FLIGHT MODE")
            mav.set_mode(custom_mode = "OFFBOARD")
        mav.set_vel(F[0], F[1], 0)



    mav.land()

run()