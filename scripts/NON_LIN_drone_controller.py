#! /usr/bin/env python3

import math
import roslib
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib
import os
import sys

import argparse

import csv,time

from insp_panels_pkg.msg import *

from   std_msgs.msg       import Float32,Int16
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative



# GLOBAL VARIABLES
x=float(0)
y=float(0)
angle=float(0)
panel_detected=float(0)

im_width = float(10000)
im_height = float(10000)
x_current = 0.0
y_current = 0.0
ground_distance=float(0)
yaw = 0.0



def get_rotation(data1):
    global x_current,y_current,yaw
    x_current  = data1.position.x
    y_current  = data1.position.y

    quat_x   = data1.orientation.x 
    quat_y   = data1.orientation.y
    quat_z   = data1.orientation.z
    quat_w   = data1.orientation.w

    orientation_list = [quat_x, quat_y, quat_z, quat_w]
        
    r, p, yaw = euler_from_quaternion (orientation_list)

def callback_loc(pose):
    global x,y,angle,panel_detected
    global im_width,im_height
    x = pose.position.x
    y = pose.position.y
    angle = pose.orientation.z 
    im_width = pose.orientation.x
    im_height = pose.orientation.y
    panel_detected = pose.orientation.w

def callback_ground(distance):
    global ground_distance
    ground_distance = distance.data

def main():
    
    global x_current,y_current,yaw
    global x,y,angle,panel_detected,ground_distance
    global im_width, im_height
    altitude=7 # meters
    P_gain_throttle=0.5

    # pubblico comando in velocita con custom message sul topic coomand
    pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

    cmd = Drone_cmd()

    rate = rospy.Rate(20) # 20hz 

    delta = .3

    while not rospy.is_shutdown():

        print("im_width/4:",im_width/4)
        print("x:",x)
        print("---")

        if x >= (im_width/4):
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0
            cmd.throttle = 0

            print("OUT OUT OUT OUT OUT OUT ")
            pub_cmd_vel.publish(cmd)
            break

        x=im_width*x/1000
        y=im_height*y/1000

        rad_angle=math.radians(angle)

        if(angle==0):
            x_line=0
            y_line=y

        else:
            m=math.tan(rad_angle)
            x_line=m*(m*x-y)/(1+m*m)
            y_line=-(m*x-y)/(1+m*m)

        if(y_line>0):
            y_perp = math.sqrt(x_line*x_line+y_line*y_line)
        else:
            y_perp = -math.sqrt(x_line*x_line+y_line*y_line)

        
        ## THIRD TECNIQUE super ganza
        
        V_x=0.001*y_line
        V_y=.2             # NOTA IMPORTANTE: Puoi scegliere qualsiasi V_y

        if(panel_detected==42): # It means rails not detected, so keep the drone still
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0
            cmd.throttle = 0
                
        
        else:
            if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
                cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)
            
            else:
                cmd.pitch =  (V_x*math.sin(rad_angle)+V_y*math.cos(rad_angle))


            if(abs(cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
                cmd.roll=5*(abs(cmd.roll)/cmd.roll)
            else: 
                if abs(y_line) < 50:
                    cmd.roll  = -((-V_x*math.cos(rad_angle)+V_y*math.sin(rad_angle)))

                else:
                    cmd.roll  = -(-V_x*math.cos(rad_angle))
            
            if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
                cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)
            else:
                cmd.yaw= 10*(rad_angle)

            if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
                cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)
            else:
                cmd.throttle = P_gain_throttle*(altitude - ground_distance)
        

        pub_cmd_vel.publish(cmd)
        '''
        print("\npanel detected: ",(panel_detected!=42)," x:",x,", y:",y,", angle:",angle,", ground distance:",ground_distance)
        print("commands: ")
        print("yaw: ", cmd.yaw)
        print("pitch: ",cmd.pitch)
        print("roll: ", cmd.roll)
        print("throttle: ", cmd.throttle)
        print("---")
        print("y_line:",y_line)
        print("z     :",ground_distance)
        '''

        with open('data_nonlin.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([cmd.yaw, cmd.pitch, cmd.roll, cmd.throttle,ground_distance,time.time()])

        rate.sleep()




if __name__ == '__main__':

    rospy.init_node('follow_line_server')

    sub_1 = rospy.Subscriber("/localization", Pose, callback_loc)
    sub_2 = rospy.Subscriber("/ground_distance", Float32, callback_ground)
    sub_3 = rospy.Subscriber('/quadrotor_pose', Pose, get_rotation)

    main()

    rospy.spin()