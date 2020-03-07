#!/usr/bin/env python
#import roslib
import rospy
import smach
import smach_ros
from MAV import MAV
import threading
from std_msgs.msg import Bool
from progress.bar import Bar



# define state Takeoff
class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.counter = 0

    def execute(self, userdata):
        global mav
        rospy.loginfo('Executing state Takeoff')
        p_bar = Bar("ROS SETUP", max=300)
        # for i in range(100):
        #     mav.arm(True)
        #     mav.set_mode(custom_mode="AUTO.TAKEOFF")
        #     mav.rate.sleep()
        # result = 'done'
    
        
        mav.set_position(0,0,0)
        for i in range(300):
            mav.local_position_pub.publish(mav.goal_pose)
            mav.rate.sleep()
            p_bar.next()
        p_bar.finish()
        rospy.loginfo("SETUP COMPLETE")
        result = mav.takeoff(5)
        return result

class SquareForever(smach.State):
    def __init__(self):
        self.aligned = Bool()
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.squareSide = 5
        self.batteryThreshold = 0.51 #30% remaining

    def execute(self, userdata):
        rospy.loginfo('\n\nRealizando trajetoria\n\n')
        height = 5
        
        x = [0, self.squareSide, self.squareSide, 0]
        y = [0, 0, self.squareSide, self.squareSide]
        
        i=0
        while not mav.battery.percentage <= self.batteryThreshold:
            mav.set_position(x[i], y[i], height)
            rospy.loginfo("Battery at: " + str(mav.battery.percentage) + "%")
            while not mav.chegou():
                rospy.loginfo("Battery at: " + str(mav.battery.percentage) + "%" + ", " + str(mav.battery.voltage) + " Volts")
                rospy.loginfo("Setting position (" + str(x[i]) + ", " + str(y[i]) + ", " + str(height) + ")")
                mav.set_position(x[i], y[i], height)
                if mav.drone_state != "OFFBOARD":
                        rospy.loginfo("SETTING OFFBOARD FLIGHT MODE")
                        mav.set_mode(custom_mode = "OFFBOARD")
                        mav.rate.sleep()
            if mav.drone_state != "OFFBOARD":
                        rospy.loginfo("SETTING OFFBOARD FLIGHT MODE")
                        mav.set_mode(custom_mode = "OFFBOARD")
                        mav.rate.sleep()
            if i < 3:
                i += 1
            else:
                i = 0

        return 'done'

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
                                transitions={'done':'SQUARE_FOREVER', 'aborted':'Mission executed successfully!'})
        smach.StateMachine.add('SQUARE_FOREVER', SquareForever(),
                                transitions={'done':'RTL', 'aborted':'RTL'})
        smach.StateMachine.add('RTL', ReturnToLand(),
                                transitions={'succeeded':'Mission executed successfully!'})

     # Execute SMACH plan
    outcome = sm.execute()
    print outcome

if __name__ == '__main__':
    main()
