#!/usr/bin/env python

import rospy
import mavros_msgs
from mavros_msgs import srv
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from mavros_msgs.msg import State
import time
import math

goal_pose = PoseStamped() #Possicao que voce deseja ir
local = PoseStamped() #capta a posicao q vc esta
drone_pose = PoseStamped() #variavel que recebe a posicao q esta e usaremos para comparacoes
current_state = State() #recebe o estado da maquina
goal_rotation = Quaternion()

#set_posicao recebe de parametros a posicao que deseja ir e publicara


def fazCirculo(R):
    rospy.init_node('Vel_Control_Node', anonymous = True)
    rate = rospy.Rate(20)
    time = 60
    end = 20.0*time
    def set_position(x, y, z):
        global goal_pose
        global goal_rotation
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation = goal_rotation
        local_position_pub.publish(goal_pose)

    #state_callback "subscrevera" e recebera o status do DRONE
    def state_callback(state_data):
        global current_state
        current_state = state_data

    #local_callback "subscrevera" e recebera a localizacao atual do DRONE
    def local_callback(local):
        global drone_pose
        drone_pose.pose.position.x = local.pose.position.x
        drone_pose.pose.position.y = local.pose.position.y
        drone_pose.pose.position.z = local.pose.position.z

        #Definicao dos publishers e subscribers
    local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 100)

    local_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_callback)
    arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

    def chegou (x,y):
        if (abs(x.pose.position.z - y.pose.position.z) < 0.1) and (abs(x.pose.position.y - y.pose.position.y) < 0.1) and (abs(x.pose.position.x - y.pose.position.x) < 0.1):
            return True
        return False

    def set_goal_rotation(theta, (x,y,z)):
        global goal_rotation
        theta = math.pi*theta/360
        if x*x + y*y + z*z != 1:
            norm = math.sqrt((x*x) + (y*y) + (z*z))
            x /= norm
            y /= norm
            z /= norm

        goal_rotation.x = math.sin(theta)*x
        goal_rotation.y = math.sin(theta)*y
        goal_rotation.z = math.sin(theta)*z
        goal_rotation.w = math.cos(theta)

    if not current_state.armed:
        arm(True)
        rate.sleep()

    part = 0.1
    i=0
    while(i<end):
        rospy.loginfo("Executing State CIRCLE")
        theta = (3/4)*math.pi + part
        #if not chegou(goal_pose, drone_pose):
        set_goal_rotation(360*(theta+(math.pi/2))/(2*math.pi), (0,0,1))
        set_position(R*math.cos(theta), R + R*math.sin(theta), h)
        part = part + 0.02
        print(theta)
        i = i + 1
        rate.sleep()
    return 'done'

h = 2
x = 0

def main():
    fazCirculo(1)


if __name__ == "__main__":
    main()
