#!/usr/bin/env python

'''

General class for Micro Air Vehicle

This class cannot be used to single MAV control, only swarms.
The swarms can contain multiple MAVROS compatible vehicles,
along with one dji Tello and one Parrot Bebop2 (in this version)

Caio Freitas - Skyrats Team
caiofreitas@usp.br

'''


import rospy
import smach
from progress.bar import ChargingBar
import smach_ros
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import Mavlink



TOL = 0.5
MAX_TIME_DISARM = 15
CONFIG = {"mavros_local_position_pub"       : "mavros/setpoint_position/local",
                "mavros_velocity_pub"       : "mavros/setpoint_velocity/cmd_vel",
                "mavros_local_atual"        : "mavros/local_position/pose",
                "mavros_state_sub"          : "mavros/state",
                "mavros_arm"                : "mavros/cmd/arming",
                "mavros_set_mode"           : "mavros/set_mode",
                "mavros_battery_sub"        : "mavros/battery",
                "mavros_extended_state"     : "mavros/extended_state",
                "tello_velocity_pub"       : "/tello/cmd_vel",
                "tello_battery_sub"        : "/tello/battery"}


class MAV:
    def __init__(self, mav_id, mav_type="mavros"):
        #rospy.init_node("MAV_" + mav_id)
        self.rate = rospy.Rate(60)

        self.drone_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.goal_vel = TwistStamped()
        self.drone_state = State()
        self.battery = BatteryState()
        ####################### Publishers ####################
        self.local_position_pub = rospy.Publisher("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_local_position_pub"]), PoseStamped, queue_size = 20)
        self.velocity_pub       = rospy.Publisher("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_velocity_pub"]),  TwistStamped, queue_size=5)

        ####################### Subscribers ###################
        self.local_atual        = rospy.Subscriber("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_local_atual"]), PoseStamped, self.local_callback)
        self.state_sub          = rospy.Subscriber("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_set_mode"]), State, self.state_callback)
        self.battery_sub        = rospy.Subscriber("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_battery_sub"]), BatteryState, self.battery_callback)
        self.extended_state_sub = rospy.Subscriber("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_extended_state"]), ExtendedState, self.extended_state_callback, queue_size=2)
        ####################### Services ######################
        self.arm                = rospy.ServiceProxy("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_arm"]), CommandBool)
        self.set_mode_srv = rospy.ServiceProxy("/uav{}/{}".format(mav_id, CONFIG[mav_type + "_set_mode"]), SetMode)

        self.LAND_STATE = ExtendedState.LANDED_STATE_UNDEFINED # landing state
        '''
        0: Undefined
        1: On ground
        2: In air
        '''

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        self.desired_state = mode
        old_mode = self.drone_state.mode
        loop_freq = 1  # Hz
        loop_rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in range(int(timeout * loop_freq)):
            if self.drone_state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    result = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not result.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                loop_rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    ###### Callback Functions ##########
    def state_callback(self, state_data):
        self.drone_state = state_data

    def set_position_target(self, type_mask, x_position=0, y_position=0, z_position=0, x_velocity=0, y_velocity=0, z_velocity=0, x_aceleration=0, y_aceleration=0, z_aceleration=0, yaw=0, yaw_rate=0):
        self.pose_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pose_target.type_mask = type_mask
        #https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
        #4095 ignores every dimension (subtract the usefull ones from it)

        self.pose_target.position.x = x_position
        self.pose_target.position.y = y_position
        self.pose_target.position.z = z_position

        self.pose_target.velocity.x = x_velocity
        self.pose_target.velocity.y = y_velocity
        self.pose_target.velocity.z = z_velocity

        self.pose_target.acceleration_or_force.x = x_aceleration
        self.pose_target.acceleration_or_force.y = y_aceleration
        self.pose_target.acceleration_or_force.z = z_aceleration

        self.pose_target.yaw = yaw
        self.pose_target.yaw_rate = yaw_rate

        self.target_pub.publish(self.pose_target)
    def battery_callback(self, bat_data):
        self.battery = bat_data

    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z

    def extended_state_callback(self, es_data):
        self.LAND_STATE = es_data.landed_state

    ####### Set Position and Velocity ################
    def set_position(self, x, y, z):
        if self.drone_state != "OFFBOARD":
            #rospy.loginfo("SETTING OFFBOARD FLIGHT MODE")
            self.set_mode_srv(custom_mode = "OFFBOARD")
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.local_position_pub.publish(self.goal_pose)
        self.rate.sleep()

    def set_vel(self, x, y, z, roll=0, pitch=0, yaw=0):
        self.goal_vel.twist.linear.x = x
        self.goal_vel.twist.linear.y = y
        self.goal_vel.twist.linear.z = z

        self.goal_vel.twist.angular.x = roll
        self.goal_vel.twist.angular.y = pitch
        self.goal_vel.twist.angular.z = yaw
        self.velocity_pub.publish(self.goal_vel)

    def chegou(self):
        if (abs(self.goal_pose.pose.position.x - self.drone_pose.pose.position.x) < TOL) and (abs(self.goal_pose.pose.position.y - self.drone_pose.pose.position.y) < TOL) and (abs(self.goal_pose.pose.position.z - self.drone_pose.pose.position.z) < TOL):
            return True
        else:
            return False

    def takeoff(self, height):
        rospy.logwarn("ARMING DRONE")
        self.arm(True)
        for i in range(3):
            self.set_mode_srv(custom_mode = "AUTO.TAKEOFF")
        return "done"


    def RTL(self):
        self.set_mode(custom_mode = "AUTO.RTL")


    def land(self):
        self.set_mode(custom_mode="AUTO.LAND")
        rospy.logwarn("LANDED_STATE: ON GROUND\nDISARMING")
        self.rate.sleep()
        if self.LAND_STATE == ExtendedState.LANDED_STATE_ON_GROUND:
            self.arm(False)
        return "succeeded"

    def _disarm(self):
        rospy.logwarn("DISARM MAV")
        if drone_pose.pose.position.z < TOL:
            for i in range(3):
                rospy.loginfo('Drone height' + str(drone_pose.pose.position.z))
                self.arm(False)
        else:
            rospy.logwarn("Altitude too high for disarming!")
            self.land()
            self.arm(False)

if __name__ == '__main__':
    mav = MAV("1") #MAV name
    mav.takeoff(3)
    mav.RTL()
