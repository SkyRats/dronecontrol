#!/usr/bin/env python
#import roslib
import rospy
import smach
import smach_ros
from MAV import MAV
import threading
from std_msgs.msg import Bool
from progress.bar import ChargingBar


# define state Takeoff
class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.counter = 0

    def execute(self, userdata):
        global mav
        rospy.loginfo('Executing state Takeoff\n')

        p_bar = ChargingBar("ROS SETUP", max=300)
        mav.set_position(0,0,0)
        for i in range(300):
            mav.local_position_pub.publish(mav.goal_pose)
            mav.rate.sleep()
            p_bar.next()
        p_bar.finish()
        rospy.loginfo("SETUP COMPLETE")
        result = mav.takeoff(5)
        return result

class Mission(smach.State):
    def __init__(self):
        self.aligned = Bool()
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.squareSide = 5
        self.batteryThreshold = 0.51 #60% remaining

    def execute(self, userdata):
        rospy.loginfo('\n\nRealizando trajetoria\n\n')
        height = 5 # 5m
        
        final_pose = (4, 3)
        
        i=0
        t=0
        T=10

        while t < 60*T:
            mav.set_position(t*final_pose[0]/(60*T), t*final_pose[1]/(60*T), 5)
            t += 1/2.0
        mav.land()
        mav.takeoff(5)
        while t < 60*T:
            mav.set_position(final_pose[0] - t*final_pose[0]/(60*T), final_pose[1] - t*final_pose[1]/(60*T), 5)
            t += 1/2.0
        mav.RTL()
        return 'done'


class ReturnToLand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        global mav
        rospy.loginfo('Executing state RTL')
        mav.RTL()
        mav.arm(False)
        return 'succeeded'


rospy.init_node('drone_state_machine', anonymous = True)

mav = MAV("1", "mavros")
def main():
    # Create a SMACH state machine
    #rospy.init_node("State Machine")
   
    sm = smach.StateMachine(outcomes=['Mission executed successfully!'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', Takeoff(),
                                transitions={'done':'MISSION', 'aborted':'Mission executed successfully!'})
        smach.StateMachine.add('MISSION', Mission(),
                                transitions={'done':'RTL', 'aborted':'RTL'})
        smach.StateMachine.add('RTL', ReturnToLand(),
                                transitions={'succeeded':'Mission executed successfully!'})

     # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo(outcome)

if __name__ == '__main__':
    main()
