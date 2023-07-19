#!/usr/bin/env python3

import numpy as np
import random
import math
import rospy
import roscpp
from   std_msgs.msg       import Float64
from   geometry_msgs.msg  import Twist
from   geometry_msgs.msg  import Pose
from   geometry_msgs.msg  import Point
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from my_custom_interfaces.msg import Drone_cmd


x_s = 0.0
y_s = 0.0
z_s = 0.0

x_current = 0.0
y_current = 0.0
z_current = 0.0

roll  = 0.0
pitch = 0.0
yaw   = 0.0

ei_ang = 0.0
ei_z   = 0.0
ei_y   = 0.0
ei_x   = 0.0

e_old_ang = 0.0
e_old_z   = 0.0
e_old_y   = 0.0
e_old_x   = 0.0



def update_olds(e_x,e_y,e_z,e_ang):

    global e_old_x,e_old_y,e_old_ang,e_old_z

    e_old_x   = e_x 
    e_old_y   = e_y
    e_old_z   = e_z
    e_old_ang = e_ang


def update_integrals(yaw_e, throttle_e,pitch_e, roll_e):

    global ei_x,ei_y,ei_z,ei_ang

    ei_ang =  ei_ang + yaw_e
    ei_z   =  ei_z + throttle_e
    ei_y   =  ei_y + pitch_e
    ei_x   =  ei_x + roll_e


def get_rotation(data1):
    global x_current, y_current, z_current, roll, pitch, yaw,old_x, old_y, old_z, old_ang

    x_current  = data1.position.x
    y_current  = data1.position.y
    z_current  = data1.position.z

    quat_x   = data1.orientation.x 
    quat_y   = data1.orientation.y
    quat_z   = data1.orientation.z
    quat_w   = data1.orientation.w

    orientation_list = [quat_x, quat_y, quat_z, quat_w]

    roll, pitch, yaw = euler_from_quaternion (orientation_list)




def get_goal(data2):
    global x_s, y_s, z_s

    x_s  = data2.x
    y_s  = data2.y
    z_s  = data2.z


def main():

    global roll, pitch, yaw, x_current,y_current,z_current

    P_gain_yaw=0.2
    D_gain_yaw=0.5
    I_gain_yaw=0#0.0001

    P_gain_throttle=0.5
    D_gain_throttle=0.1
    I_gain_throttle=0#0.0001

    P_gain_pitch=P_gain_roll=0.2
    D_gain_pitch=D_gain_roll=0.2
    I_gain_pitch=I_gain_roll=0#0.0001

    rospy.init_node('quad_waypoint2', anonymous=True)

    rate = rospy.Rate(10) # 5hz

    rospy.loginfo("PID positional controller active")

    # aggiunto io per prendere la posizione del drone dalla simulazione 
    sub = rospy.Subscriber('/quadrotor_pose', Pose, get_rotation)

    # sub per vedere la posi<ione iniziale
    sub = rospy.Subscriber('/pose_goal', Point, get_goal)

    # pubblico comando in velocita con custom message sul topic coomand
    pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

    cmd =Drone_cmd()

    rospy.wait_for_message('/pose_goal', Point)
    ang_s = 0

    while not rospy.is_shutdown():

        #print("roll, pitch, yaw: ",x_current, y_current, yaw)

        e_x = x_s - x_current
        e_y = y_s - y_current
        e_z = z_s - z_current
        e_ang = ang_s - yaw

        ed_x = e_x - e_old_x
        ed_y = e_y - e_old_y
        ed_z = e_z - e_old_z
        ed_ang = e_ang - e_old_ang

        if (e_ang+e_z+e_y+e_x) > 1.5:

            update_integrals(e_ang,e_z,e_y,e_x)

        

        cmd.roll     = P_gain_pitch  *   e_y   + D_gain_pitch *       ed_y   + I_gain_pitch  *    ei_y

        if(abs(cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.roll=5*(abs(cmd.roll)/cmd.roll)

        cmd.pitch    = -(P_gain_roll  *    e_x   + D_gain_roll *        ed_x   + I_gain_roll  *     ei_x)

        if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)

        cmd.throttle = P_gain_throttle * e_z   + D_gain_throttle *    ed_z   + I_gain_throttle  * ei_z

        if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
            cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)

        cmd.yaw      = P_gain_yaw *      e_ang + D_gain_yaw *         ed_ang + I_gain_yaw *       ei_ang

        if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
            cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)



        update_olds(e_x,e_y,e_z,e_ang)

        pub_cmd_vel.publish(cmd)
    rate.sleep()
    rospy.loginfo("Node is shutting down")

    cmd.yaw = 0
    cmd.pitch = 0
    cmd.roll = 0
    cmd.throttle = 0
    rate.sleep()
    rate.sleep()
    rate.sleep()
    pub_cmd_vel.publish(cmd)

    # rospy.spin()

if __name__ == '__main__':
    main()