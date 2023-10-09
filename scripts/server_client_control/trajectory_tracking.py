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

from insp_panels_pkg.msg import *

import datetime

T = 0.0

def poly_line_coef(s_i,s_f,v_i,v_f,a_i,a_f,v_max):
    global T
    cv = 1.875
    T = (cv/v_max)* s_f
    a_0= s_i
    a_1= v_i
    a_2= 1/2 * a_i
    a_3= 1/2 * 1/(pow(T,3))*(20*s_f-(8*v_f+12*v_i)*T-(3*a_i-a_f)*pow(T,2))
    a_4= 1/2 * 1/(pow(T,4))*(-30*s_f+(14*v_f+16*v_i)*T+(3*a_f-2*a_i)*pow(T,2))
    a_5= 1/2 * 1/(pow(T,5))*(12*s_f-6*(v_f+v_i)*T+(a_f-a_i)*pow(T,2))
    return a_0,a_1,a_2,a_3,a_4,a_5

def main():

    rospy.init_node('traj_track', anonymous=True)

    rate = rospy.Rate(20) # 5hz

    rospy.loginfo("Trajectory")

    # pubblico comando in velocita con custom message sul topic coomand
    pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

    cmd =Drone_cmd()

    a_0_x,a_1_x,a_2_x,a_3_x,a_4_x,a_5_x = poly_line_coef(0,.7,0,0,0,0,0.2)
    #a_0_y,a_1_y,a_2_y,a_3_y,a_4_y,a_5_y = poly_line_coef(0,1,0,0,0,0,0.2)

    check = datetime.datetime.now()+ datetime.timedelta(seconds = T)
    t_0 = datetime.datetime.now()
    print("T: ",T)
    print( a_0_x,a_1_x,a_2_x,a_3_x,a_4_x,a_5_x)
    #print(a_0_y,a_1_y,a_2_y,a_3_y,a_4_y,a_5_y)
    while not rospy.is_shutdown():
        t = datetime.datetime.now()
        cmd.yaw = 0
        cmd.pitch = 0#a_1_y+ a_2_y*2*(t-t_0).total_seconds()+ a_3_y*3*pow((t-t_0).total_seconds(),2)+ a_4_y*4*pow((t-t_0).total_seconds(),3)+ a_5_y*5*pow((t-t_0).total_seconds(),4)
        cmd.roll = a_1_x+ a_2_x*2*(t-t_0).total_seconds()+ a_3_x*3*pow((t-t_0).total_seconds(),2)+ a_4_x*4*pow((t-t_0).total_seconds(),3)+ a_5_x*5*pow((t-t_0).total_seconds(),4)
        cmd.throttle = 0
        #print("cmd.pitch: ",cmd.pitch)
        #print("cmd.roll: ",cmd.roll)
        pub_cmd_vel.publish(cmd)

        if datetime.datetime.now()>check:
            break

        rate.sleep()
    rospy.loginfo("Node is shutting down")

    # rospy.spin()

if __name__ == '__main__':
    main()