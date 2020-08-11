import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from MAV import MAV

laser_data = LaserScan()

MASK_VELOCITY = 0b0000011111000111

# def laser_callback(data):
#     global laser_data
#     laser_data = data

def run():
    rospy.init_node("avoidance")
    #laser_sub = rospy.Subscriber("/laser/scan", LaserScan, laser_callback, queue_size=1)
    mav = MAV("1")
    goal = np.array([4, 0])
    initial_height = 3
    mav.takeoff(initial_height)
    Kr = 0.2 # repulsive
    Ka = 0.5 #attractive
    Kz = 0.5
    d = (mav.drone_pose.pose.position.x - goal[0])**2 + (mav.drone_pose.pose.position.y - goal[1])**2
    d = np.sqrt(d)
    
        
    while not rospy.is_shutdown() and d > 0.3:
        d = (mav.drone_pose.pose.position.x - goal[0])**2 + (mav.drone_pose.pose.position.y - goal[1])**2
        d = np.sqrt(d)
        euler_orientation = euler_from_quaternion(ros_numpy.numpify(mav.drone_pose.pose.orientation))
        ########################theta_goal global###################################
        deltaY = goal[1] - mav.drone_pose.pose.position.y
        deltaX = goal[0] - mav.drone_pose.pose.position.x
        if deltaY > 0 and deltaX >= 0:
            if deltaX == 0:
                theta_goal = 1.57079632679
            else:
                theta_goal = np.arctan(deltaY/deltaX)
        if deltaY >= 0 and deltaX < 0:
            if deltaY == 0:
                theta_goal = 3.14
            else:
                theta_goal = np.arctan(abs(deltaX/deltaY)) + 1.57079632679 #90
        if deltaY < 0 and deltaX <= 0:
            if deltaX == 0:
                theta_goal = -1.57079632679
            else:
                theta_goal = -1*np.arctan(abs(deltaX/deltaY)) - 1.57079632679 #180
        if deltaY <= 0 and deltaX > 0:
            if deltaY == 0:
                theta_goal = 0
            else:
                theta_goal = -1*np.arctan(abs(deltaY/deltaX))
        ##################################################################################
        if theta_goal - euler_orientation[2] >= 0:
            theta_rel = theta_goal - euler_orientation[2]
        else: 
            theta_rel = 6.28318530718 + (theta_goal - euler_orientation[2])
        theta = laser_data.angle_min
        Ft = np.array([0.0, 0.0])
        Fg = np.array([Ka*d*np.cos(theta_rel),
                                    Ka*d*np.sin(theta_rel)])

        F = Ft + Fg
        #rospy.loginfo("Force = {}".format(F))
        mav.set_position_target(type_mask=MASK_VELOCITY,
                                x_velocity=F[0],
                                y_velocity=F[1],
                                z_velocity=Kz*(initial_height - mav.drone_pose.pose.position.z),
                                yaw_rate=0)


    mav.land()

run()