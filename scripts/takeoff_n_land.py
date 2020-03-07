#!/usr/bin/env python
#import roslib
import rospy
import smach
import smach_ros
from MAV import MAV
import threading
import time
#from align_reference import adjust_position
from std_msgs.msg import Bool, Float32
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue


rospy.init_node("state_machine")
mav = MAV("1")

# define state Takeoff
class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.counter = 0

    def execute(self, userdata):
        global mav
        rospy.loginfo('Executing state Takeoff')
        mav.set_position(0,0,0)
        for i in range(200):
            mav.local_position_pub.publish(mav.goal_pose)
            mav.rate.sleep()
        rospy.loginfo("SETUP COMPLETE")
        result = mav.takeoff(3)
        return result



class Stay(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.msg_param = ParamValue(integer=0, real=10.9)
        self.param_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)

    def execute(self, userdata):
        global mav
        rospy.loginfo('Executing state Stay')

        init_time = rospy.Time.now()

        while not rospy.Time.now() - init_time > rospy.Duration(120): # 2 minutes
            mav.set_position(mav.drone_pose.pose.position.x,
                            mav.drone_pose.pose.position.y,
                            mav.drone_pose.pose.position.z)
            mav.rate.sleep()
            self.param_srv.call("MPC_XY_P", self.msg_param)
        return "succeeded"

 # define state RTL
class ReturnToLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        global mav
        rospy.loginfo('Executing state RTL')
        mav.RTL()
        mav.arm(False)
        return 'succeeded'


#rospy.init_node('drone_state_machine', anonymous = True)
#rate = rospy.Rate(60) # 10hz

def run():
    # Create a SMACH state machine
    # rospy.init_node("state_machine")
    # mav = MAV("1")
    sm = smach.StateMachine(outcomes=['Mission executed successfully!'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', Takeoff(),
                                transitions={'done':'STAY', 'aborted':'Mission executed successfully!'})
        smach.StateMachine.add('STAY', Stay(),
                                transitions={'succeeded':'RTL'})
        smach.StateMachine.add('RTL', ReturnToLand(),
                                transitions={'succeeded':'Mission executed successfully!'})

     # Execute SMACH plan
    outcome = sm.execute()
    print outcome

if __name__ == '__main__':
    run()
