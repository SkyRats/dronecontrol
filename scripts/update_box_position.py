#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkState, LinkStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import PoseStamped, Pose, Wrench, Vector3, Twist
import ros_numpy
import numpy as np
def pose_callback(lsts):
    global pose, links_pub, count
    #linkstate.pose = lsts.pose[2] + lsts.pose[8] #base link for vehicle 1
    #print("pose[2]: {}".format(lsts.pose[2]))
    #print("pose[8]: {}".format(lsts.pose[8]))
    avg = (ros_numpy.numpify(lsts.pose[2]) + ros_numpy.numpify(lsts.pose[8]))/2
    posemsg = ros_numpy.msgify(Pose, avg)

    #avgtw = (ros_numpy.numpify(lsts.twist[2]) + ros_numpy.numpify(lsts.twist[8]))/2
    twistmsg = lsts.twist[2]

    posemsg.position.z -= 0.5
    linkstate.pose = posemsg
    linkstate.twist = twistmsg
    
    #links_pub.publish(linkstate)

    d1_vec = (ros_numpy.numpify(lsts.pose[2]) - (ros_numpy.numpify(lsts.pose[2]) + ros_numpy.numpify(lsts.pose[8]))/2)
    d1 = np.sqrt(d1_vec[0][3]*d1_vec[0][3] + d1_vec[1][3]*d1_vec[1][3] + d1_vec[2][3]*d1_vec[2][3])

    l1_vec = ros_numpy.numpify(lsts.pose[2]) - ros_numpy.numpify(lsts.pose[1])
    x1 = l1_vec[0][3]
    y1 = l1_vec[1][3]
    z1 = l1_vec[2][3]
    l1 = np.sqrt(x1*x1 + y1*y1 + z1*z1)
    
    theta1 = np.arcsin(d1/l1)
    print("Theta1 = {} deg".format(180*theta1/np.pi))
    
    d2_vec = (ros_numpy.numpify(lsts.pose[2]) - (ros_numpy.numpify(lsts.pose[2]) + ros_numpy.numpify(lsts.pose[8]))/2)
    d2 = np.sqrt(d1_vec[0][3]*d1_vec[0][3] + d1_vec[1][3]*d1_vec[1][3] + d1_vec[2][3]*d1_vec[2][3])
    l2_vec = ros_numpy.numpify(lsts.pose[2]) - ros_numpy.numpify(lsts.pose[1])
    x2 = l2_vec[0][3]
    y2 = l2_vec[1][3]
    z2 = l2_vec[2][3]
    l2 = np.sqrt(x2*x2 + y2*y2 + z2*z2)
    
    theta2 = np.arcsin(d1/l1)
    #print("Theta2 = {} deg".format(180*theta1/np.pi))

    m = 1 # 1kg
    g = 9.8 #9.8m/s/s
    Fz = (m*g/2)
    print("force1 = ({}, {}, {})".format(-Fz*np.tan(theta1), 0, -Fz))
    print("force2 = ({}, {}, {})".format(Fz*np.tan(theta2), 0, -Fz))
    force1.wrench = Wrench(force=Vector3(Fz*np.tan(theta1), 0, -Fz))
    force2.wrench = Wrench(force=Vector3(-Fz*np.tan(theta2), 0, -Fz))
    if theta1 != 'nan' and theta2 != 'nan':
        if count == 50:
            links_pub.publish(linkstate)
            count=0
        else:
            count += 1
        apply_force("iris_0::base_link", "iris_0::base_link", Vector3(0,0,0), force1.wrench, rospy.Time(0), rospy.Duration(-1))
        apply_force("iris_1::base_link", "iris_1::base_link", Vector3(0,0,0), force2.wrench, rospy.Time(0), rospy.Duration(-1))
    #apply_force(force1)
    #print(lsts.link_name)
    #print(lsts)

pose = PoseStamped()
count = 0

force1 = ApplyBodyWrench()
force1.body_name = "iris_0::base_link"
force1.reference_frame = "iris_0::base_link"
force2 = ApplyBodyWrench()

force2.body_name = "iris_1::base_link"
force2.reference_frame = "iris_0::base_link"
linkstate = LinkState()
linkstate.link_name = "my_model::link"
linkstate.reference_frame = "asphalt_plane::link"

#links_sub = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, pose_callback)
links_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, pose_callback)
links_pub = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)
apply_force = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

def main():
    rospy.init_node("pose_updater")
    rospy.spin()

if __name__ == "__main__":
    main()
