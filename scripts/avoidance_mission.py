import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from MAV import MAV

laser_data = LaserScan()

MASK_VELOCITY = 0b0000011111000111

def laser_callback(data):
    global laser_data
    laser_data = data

def run():
    rospy.init_node("avoidance")
    laser_sub = rospy.Subscriber("/laser/scan", LaserScan, laser_callback, queue_size=1)
    mav = MAV("1")
    goal = np.array([8, 0])
    initial_height = 1.5
    mav.takeoff(initial_height)
    Kr = 0.2 # repulsive
    Ka = 0.3 #attractive
    Kz = 0.1
    
    d = (mav.drone_pose.pose.position.x - goal[0])**2 + (mav.drone_pose.pose.position.y - goal[1])**2
    d = np.sqrt(d)
    roll, pitch, yaw = euler_from_quaternion([mav.drone_pose.pose.orientation.x,
                                                mav.drone_pose.pose.orientation.y,
                                                mav.drone_pose.pose.orientation.z,
                                                mav.drone_pose.pose.orientation.w])
    if mav.drone_pose.pose.orientation.w == 0.0:
        theta_goal = 0.0
    else:
        theta_goal = np.atan((goal[1] - mav.drone_pose.pose.position.y)/(goal[0] - mav.drone_pose.pose.position.x))
        if mav.drone_pose.pose.x > goal[0]:
            theta_goal *= -1
    while not rospy.is_shutdown():
        d = (mav.drone_pose.pose.position.x - goal[0])**2 + (mav.drone_pose.pose.position.y - goal[1])**2
        d = np.sqrt(d)
        if mav.drone_pose.pose.orientation.w == 0.0:
            theta_goal = 0.0
        else:
            theta_goal = np.atan((goal[1] - mav.drone_pose.pose.position.y)/(goal[0] - mav.drone_pose.pose.position.x))
            if mav.drone_pose.pose.x > goal[0]:
                theta_goal *= -1
        theta = laser_data.angle_min
        Ft = np.array([0.0, 0.0])
        Fg = np.array([Ka*d*np.cos(theta_goal - mav.drone_pose.pose.orientation.w),
                                    Ka*d*np.sin(theta_goal- mav.drone_pose.pose.orientation.w)])
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
        mav.set_position_target(type_mask=MASK_VELOCITY,
                                x_velocity=F[0],
                                y_velocity=F[1],
                                z_velocity=Kz*(initial_height - mav.drone_pose.pose.position.z),
                                yaw_rate=0)



    mav.land()

run()