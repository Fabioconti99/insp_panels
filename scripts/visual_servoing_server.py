#! /usr/bin/env python3

import math
import roslib
import numpy as np
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

from insp_panels_pkg.msg import *

from   std_msgs.msg       import Float32,Int16
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler


# pubblico comando in velocita con custom message sul topic coomand
pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

x_line = 0
y_line = 0 


class FollowLineServer:


    _feedback = FollowLineFeedback() 


    _result = FollowLineResult()

    def __init__(self):

        self.server = actionlib.SimpleActionServer('follow_line', FollowLineAction, self.execute, False)

        self.x_im = 0
        # GLOBAL VARIABLES
        self.rho = float(0)
        self.theta=float(0)
        self.rail_detected=float(0)

        self.z = float(0)

        self.y_current = float(0)
        self.x_current = float(0)

        self.im_width = float (10000)

        self.x_s = float(0)
        self.y_s = float(0)

        self.delta = float(0.3)

        self.server.start()

    def callback_loc(self, pose):

        global x_line,y_line

        self.im_width=pose.orientation.x
        im_height = pose.orientation.y

        self.x_im = pose.position.x
        y_im = pose.position.y

        self.x_im = self.im_width*self.x_im/1000
        y_im = im_height*y_im/1000

        if(math.radians(pose.orientation.z)==0):
            x_line=0
            y_line=(y_im)

        else:
            m=math.tan(math.radians(pose.orientation.z))
            x_line=m*(m*pose.position.x-y_im)/(1+m*m)
            y_line=-(m*pose.position.x-y_im)/(1+m*m)

        if pose.orientation.z > 0:

            if x_line < 0:
                self.theta = math.pi-math.radians(90-abs(pose.orientation.z))

            else :
                self.theta = math.radians(90-abs(pose.orientation.z))

        elif pose.orientation.z < 0:

            if x_line >= 0:
                self.theta = math.radians(90-abs(pose.orientation.z))

            else:
                self.theta = math.pi-math.radians(90-abs(pose.orientation.z))
        else:

            if y_line > 0:
                self.theta = math.radians(90)

            else:
                self.theta = -math.radians(90)

        if y_line < 0:    
            self.rho = -math.sqrt((x_line*x_line)+(y_line*y_line))
        else:    
            self.rho = math.sqrt((x_line*x_line)+(y_line*y_line))


        self.rail_detected = pose.orientation.w


    def callback_ground(self,distance):

        self.z = distance.data

    def execute(self, goal):

        global y_line


        lam = np.array([[0.1,0,0],[0,0.01,0],[0,0,0.005]])
        #lam = np.array([[1,0,0,0],[0,1,0,0],[0,0,0.01,0],[0,0,0,0.005]])

        v_y_s = float(0.2)
        rho_s = float(0)
        theta_s =float(math.radians(90))
        z_s=float(7) # meters


        s_star = np.array([[rho_s],[theta_s],[z_s]])

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
        A_d = np.array([[1,0,0],[0,0,0],[0,1,0],[0,0,0],[0,0,0],[0,0,1]])

        J_ = c_W_b @ A_d

        A = 0
        B = 0
        C = 1
        D = -1
        l_rho = float((A*self.rho*math.cos(self.theta)+B*self.rho*math.sin(self.theta)+C)/D)
        l_theta = float((A*math.sin(self.theta)-B*math.cos(self.theta))/D)

        while not rospy.is_shutdown():

            self.x_s = goal.x
            self.y_s = goal.y

            if self.x_im >= (self.im_width/4):
                cmd.yaw = 0
                cmd.pitch = 0
                cmd.roll = 0
                cmd.throttle = 0

                print("OUT OUT OUT OUT OUT OUT ")
                pub_cmd_vel.publish(cmd)
                break
            
            ## FOURTH TECNIQUE 

            # Ax+By+Cz+D = 0

            L_line = np.array([[l_rho*math.cos(self.theta),l_rho*math.sin(self.theta),-l_rho*self.rho, (1+self.rho*self.rho)*math.sin(self.theta), -(1+self.rho*self.rho)*math.cos(self.theta),0],\
                [l_theta*math.cos(self.theta),l_theta*math.sin(self.theta),-l_theta * self.rho, -self.rho*math.cos(self.theta),-self.rho*math.sin(self.theta),-1]])
            
            

            J_s_pseudo = np.linalg.pinv(np.vstack((L_line @ J_,[[0,1,0]])))

            s_star = np.array([[rho_s],[theta_s],[z_s]])
            s = np.array([[self.rho],[self.theta],[self.z]])

            U = -lam @ J_s_pseudo @ (s-s_star)

            if y_line<0:
                cmd.roll = -abs(U[0,0])*0.01    # movimento sulle x

            if y_line>0:
                cmd.roll = abs(U[0,0])*0.01    # movimento sulle x


            if goal.mod == 0:
                self.cmd.yaw = 0
                self.cmd.pitch = 0
                self.cmd.roll = 0
                self.cmd.throttle = 0

            elif goal.mod == 1:

                cmd.pitch = v_y_s

            elif goal.mod == 2:

                cmd.pitch = -v_y_s

            else:
                print("---Not Valid---")

            cmd.throttle = U[1,0]
            cmd.yaw = U[2,0]     # movimento sullo yaw

            
            if(abs(cmd.throttle)>4): # MAX throttle DJI= 4m/s
                cmd.throttle=4*(abs(cmd.throttle)/cmd.throttle)

            if(abs(cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
                cmd.yaw=30*(abs(cmd.yaw)/cmd.yaw)
                #self.cmd.yaw= -30*(abs(self.cmd.yaw)/self.cmd.yaw)
            
            if(abs(cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
                cmd.roll=5*(abs(cmd.roll)/cmd.roll)

            if(abs(cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
                cmd.pitch=5*(abs(cmd.pitch)/cmd.pitch)
                #self.cmd.pitch= - 5*(abs(self.cmd.pitch)/self.cmd.pitch)
            

            if(self.rail_detected==42): # It means rails not detected, so keep the drone still
                cmd.yaw = 0
                cmd.pitch = 0
                cmd.roll = 0
                cmd.throttle = 0

            pub_cmd_vel.publish(cmd)

            #print("\nrail detected: ",(self.rail_detected!=42)," x:",self.x,", y:",self.y,", angle:",self.theta,", ground distance:",self.z)
            print("---")
            print("commands: ")
            print("yaw     : ", cmd.yaw)
            print("pitch   : ", cmd.pitch)
            print("roll    : ", cmd.roll)
            print("throttle: ", cmd.throttle)

            print("---")
            print("Data    :")
            print("y_line  :",y_line)
            print("z       :",self.z)
            print("theta   :",self.theta)
            #print("v_y     :",self.v_y)

            self._feedback.x = self.x_current
            self._feedback.y = self.y_current
            self._feedback.z = self.z

            self.server.publish_feedback(self._feedback)
            rate.sleep()

        cmd.roll     = 0.0
        cmd.pitch    = 0.0
        cmd.yaw      = 0.0
        cmd.throttle = 0.0

        pub_cmd_vel.publish(cmd)

        self.server.set_succeeded(self._result)




if __name__ == '__main__':

    rospy.init_node('follow_line_server')

    server = FollowLineServer()

    sub_1 = rospy.Subscriber("/localization", Pose, server.callback_loc)
    sub_2 = rospy.Subscriber("/ground_distance", Float32, server.callback_ground)
    rospy.spin()