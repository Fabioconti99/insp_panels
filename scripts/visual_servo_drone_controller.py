#! /usr/bin/env python3

import math
import os
import roslib
import numpy as np
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

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

# pubblico comando in velocita con custom message sul topic coomand
pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

x_line = 0
y_line = 0 

x_im = 0
# GLOBAL VARIABLES
rho = float(0)
theta=float(0)
rail_detected=float(0)

z = float(0)

v_y = float(0)

y_current = float(0)

im_width = float (10000)

def get_rotation(data1):
    global y_current
    y_current  = data1.position.y

def callback_loc(pose):
    global x_line,y_line
    global theta,rho,rail_detected
    global x_im,im_width
    im_width=pose.orientation.x
    x_im = pose.position.x
    if(math.radians(pose.orientation.z)==0):
        x_line=0
        y_line=(pose.position.y)

    else:
        m=math.tan(math.radians(pose.orientation.z))
        x_line=m*(m*pose.position.x-pose.position.y)/(1+m*m)
        y_line=-(m*pose.position.x-pose.position.y)/(1+m*m)

    if pose.orientation.z > 0:

        if x_line < 0:
            theta = math.pi-math.radians(90-abs(pose.orientation.z))

        else :
            theta = math.radians(90-abs(pose.orientation.z))

    elif pose.orientation.z < 0:

        if x_line >= 0:
            theta = math.radians(90-abs(pose.orientation.z))

        else:
            theta = math.pi-math.radians(90-abs(pose.orientation.z))
    else:

        if y_line > 0:
            theta = math.radians(90)

        else:
            theta = -math.radians(90)

    if y_line < 0:    
        rho = -math.sqrt((x_line*x_line)+(y_line*y_line))
    else:    
        rho = math.sqrt((x_line*x_line)+(y_line*y_line))


    rail_detected = pose.orientation.w


def callback_ground(distance):
    global z
    z = distance.data

def main():

    global rho,theta
    global y_line

    lam = np.array([[0.1,0,0,0],[0,1,0,0],[0,0,0.01,0],[0,0,0,0.005]])
    #lam = np.array([[1,0,0,0],[0,1,0,0],[0,0,0.01,0],[0,0,0,0.005]])

    y_prec = float(0)

    v_y_s = float(0.2)
    rho_s = float(0)
    theta_s =float(math.radians(90))
    z_s=float(7) # meters


    s_star = np.array([[rho_s],[theta_s],[z_s],[v_y_s]])

    rate = rospy.Rate(20) # 20hz 

    cmd =Drone_cmd()

    v_y_s = .2

    #c_R_b = np.array([[0,-1,0],[math.cos(math.radians(35)),0,math.sin(math.radians(35))],[-math.sin(math.radians(35)),0,math.cos(math.radians(35))]])
    c_R_b = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    c_t_b = np.array([[0,0.01,0],[-0.01,0,-0.1],[0,0.1,0]])

    t_R = c_t_b @ c_R_b

    #c_W_b = np.vstack((np.hstack((c_R_b,t_R)),np.hstack((np.zeros((3,3)),c_R_b))))
    c_W_b = np.array([[c_R_b[0,0],c_R_b[0,1],c_R_b[0,2],t_R[0,0],t_R[0,1],t_R[0,2]],[c_R_b[1,0],c_R_b[1,1],c_R_b[1,2],t_R[1,0],t_R[1,1],t_R[1,2]],[c_R_b[2,0],c_R_b[2,1],c_R_b[2,2],t_R[2,0],t_R[2,1],t_R[2,2]],[0,0,0,c_R_b[0,0],c_R_b[0,1],c_R_b[0,2]],[0,0,0,c_R_b[1,0],c_R_b[1,1],c_R_b[1,2]],[0,0,0,c_R_b[2,0],c_R_b[2,1],c_R_b[2,2]]])

    #Vx,Vy,Vz,Wz
    A_d = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]])

    J_ = c_W_b @ A_d

    A = 0
    B = 0
    C = 1
    D = -1
    l_rho = float((A*rho*math.cos(theta)+B*rho*math.sin(theta)+C)/D)
    l_theta = float((A*math.sin(theta)-B*math.cos(theta))/D)

    while not rospy.is_shutdown():

        if x_im >= (im_width/4):
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0
            cmd.throttle = 0

            print("OUT OUT OUT OUT OUT OUT ")
            pub_cmd_vel.publish(cmd)
            break

        v_y = (y_current-y_prec)/(1/20)
        y_prec = y_current
        
        ## FOURTH TECNIQUE 

        # Ax+By+Cz+D = 0

        L_line = np.array([[l_rho*math.cos(theta),l_rho*math.sin(theta),-l_rho*rho, (1+rho*rho)*math.sin(theta), -(1+rho*rho)*math.cos(theta),0],\
            [l_theta*math.cos(theta),l_theta*math.sin(theta),-l_theta * rho, -rho*math.cos(theta),-rho*math.sin(theta),-1]])
        
        

        J_s_pseudo = np.linalg.pinv(np.vstack((L_line @ J_,[[0,0,1,0],[0,1,0,0]])))

        s_star = np.array([[rho_s],[theta_s],[z_s],[v_y_s]])
        s = np.array([[rho],[theta],[z],[v_y]])

        U = -lam @ J_s_pseudo @ (s-s_star)

        if y_line<0:
            cmd.roll = -abs(U[0,0])*0.01    # movimento sulle x

        if y_line>0:
            cmd.roll = abs(U[0,0])*0.01    # movimento sulle x


        cmd.pitch = U[1,0]
        cmd.throttle = U[2,0]
        cmd.yaw = U[3,0]     # movimento sullo yaw

        
        if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
            cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)

        if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
            cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)
        
        if(abs(cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.roll=5*(abs(cmd.roll)/cmd.roll)

        if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
            cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)
        

        if(rail_detected==42): # It means rails not detected, so keep the drone still
            cmd.yaw = 0
            cmd.pitch = 0
            cmd.roll = 0
            cmd.throttle = 0


        pub_cmd_vel.publish(cmd)

        #print("\nrail detected: ",(rail_detected!=42)," x:",x,", y:",y,", angle:",theta,", ground distance:",z)
        print("---")
        print("commands: ")
        print("yaw     : ", cmd.yaw)
        print("pitch   : ", cmd.pitch)
        print("roll    : ", cmd.roll)
        print("throttle: ", cmd.throttle)

        print("---")
        print("Data    :")
        print("y_line  :",y_line)
        print("z       :",z)
        print("theta   :",theta)
        print("v_y     :",v_y)

        rate.sleep()

    cmd.roll     = 0.0
    cmd.pitch    = 0.0
    cmd.yaw      = 0.0
    cmd.throttle = 0.0

    pub_cmd_vel.publish(cmd)




if __name__ == '__main__':

    rospy.init_node('follow_line_server')

    sub_1 = rospy.Subscriber("/localization", Pose, callback_loc)
    sub_2 = rospy.Subscriber("/ground_distance", Float32, callback_ground)
    sub_3 = rospy.Subscriber('/quadrotor_pose', Pose, get_rotation)

    main()

    rospy.spin()