#! /usr/bin/env python3

import math
import roslib
import numpy as np
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

from insp_rail_pkg.msg import *

from   std_msgs.msg       import Float32,Int16
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from my_custom_interfaces.msg import Drone_cmd


# pubblico comando in velocita con custom message sul topic coomand
pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel

x_line = 0
y_line = 0 


class FollowLineServer:


    _feedback = FollowLineFeedback() 


    _result = FollowLineResult()

    def __init__(self):

        self.server = actionlib.SimpleActionServer('follow_line', FollowLineAction, self.execute, False)

        # GLOBAL VARIABLES
        self.rho = float(0)
        self.theta=float(0)
        self.rail_detected=float(0)

        self.z = float(0)

        self.x_s = float(0)
        self.y_s = float(0)
        self.z_s=float(7) # meters

        # S_s vector 

        self.v_y_s = float(0.2)
        self.rho_s = float(0)
        self.theta_s =float(math.radians(90))

        self.P_z=float(0.5)
        self.lam = float(0.01)

        self.x_current = float(0)
        self.y_prec = float(0)
        self.y_current = float(0)
        self.yaw = float(0)
        self.v_y = float(0)

        self.mod= int(0)
        self.flag= int(0)

        # Two control variables
        #self.U=np.array([[0.0],[0.0]])

        # three control variables
        #self.U=np.array([[0.0],[0.0],[0.0]])

        # four control variables
        #self.U=np.array([[0.0],[0.0],[0.0],[0.0]])

        self.cmd =Drone_cmd()

        self.server.start()

    def get_rotation(self,data1):

        self.x_current  = data1.position.x
        self.y_current  = data1.position.y

        quat_x   = data1.orientation.x 
        quat_y   = data1.orientation.y
        quat_z   = data1.orientation.z
        quat_w   = data1.orientation.w

        orientation_list = [quat_x, quat_y, quat_z, quat_w]
        
        r, p, self.yaw = euler_from_quaternion(orientation_list)

    def callback_loc(self, pose):
        global x_line,y_line

        if pose.orientation.z > 0:
            self.theta = -math.radians(90-abs(pose.orientation.z))

        elif pose.orientation.z < 0: 
            self.theta = math.radians(90-abs(pose.orientation.z))
        else:
            self.theta = math.radians(90)

        self.rail_detected = pose.orientation.w

        if(self.theta==0):
            x_line=0
            y_line=pose.position.y

        else:
            m=math.tan(-math.pi/2-self.theta)
            x_line=m*(m*pose.position.x-pose.position.y)/(1+m*m)
            y_line=-(m*pose.position.x-pose.position.y)/(1+m*m)

        self.rho = math.sqrt((x_line*x_line)+(y_line*y_line))

    def callback_ground(self, distance):

        self.z = distance.data

    def execute(self, goal):

        global x_line,y_line
        global pub_cmd_vel

        rate = rospy.Rate(20) # 20hz 

        delta = .3

        c_R_b = np.array([[0,-1,0],[1,0,0],[0,0,1]])

        c_t_b = np.array([[0,0.01,0],[-0.01,0,-0.1],[0,0.1,0]])

        t_R = c_t_b @ c_R_b

        c_W_b = np.array([[c_R_b[0,0],c_R_b[0,1],c_R_b[0,2],t_R[0,0],t_R[0,1],t_R[0,2]],[c_R_b[1,0],c_R_b[1,1],c_R_b[1,2],t_R[1,0],t_R[1,1],t_R[1,2]],[c_R_b[2,0],c_R_b[2,1],c_R_b[2,2],t_R[2,0],t_R[2,1],t_R[2,2]],[0,0,0,c_R_b[0,0],c_R_b[0,1],c_R_b[0,2]],[0,0,0,c_R_b[1,0],c_R_b[1,1],c_R_b[1,2]],[0,0,0,c_R_b[2,0],c_R_b[2,1],c_R_b[2,2]]])

        # Vx,Vy,Wz
        #A_d = np.array([[1,0,0],[0,1,0],[0,0,0],[0,0,0],[0,0,0],[0,0,1]])

        # Vx,Wz
        #A_d = np.array([[1,0],[0,0],[0,0],[0,0],[0,0],[0,1]])

        #Vx,Vz,Wz
        #A_d = np.array([[1,0,0],[0,0,0],[0,1,0],[0,0,0],[0,0,0],[0,0,1]])

        #Vx,Vy,Vz,Wz
        A_d = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]])

        J_ = c_W_b @ A_d

        self.x_s = goal.x
        self.y_s = goal.y

        i = 0

        A = 0
        B = 0
        C = 1
        D = -1
        l_rho = float((A*self.rho*math.cos(self.theta)+B*self.rho*math.sin(self.theta)+C)/D)
        l_theta = float((A*math.sin(self.theta)-B*math.cos(self.theta))/D)

        #s_star = np.array([[self.rho_s],[self.theta_s],[self.v_y_s]])

        #s_star = np.array([[self.rho_s],[self.theta_s]])

        #s_star = np.array([[self.rho_s],[self.z_s],[self.theta_s]])

        s_star = np.array([[self.rho_s],[self.theta_s],[self.z_s],[self.v_y_s]])

        while not rospy.is_shutdown():

            self.x_s = goal.x
            self.y_s = goal.y

            self.v_y = (self.y_current-self.y_prec)/(1/20)
            self.y_prec = self.y_current

            if i>3 and self.x_current < (self.x_s+delta) and self.x_current > (self.x_s-delta) and self.y_current < (self.y_s+delta) and self.y_current > (self.y_s-delta):

                print(self.x_current,">",self.x_s+delta," ",self.x_current,"<",self.x_s-delta)
                print(self.y_current,">",self.y_s+delta," ",self.y_current,"<",self.y_s-delta)

                break

            if self.server.is_preempt_requested():

                rospy.loginfo('Preempted')

                #self.server.set_preempted()

                break

            
            ## FORTH TECNIQUE super ganza

            # Ax+By+Cz+D = 0

            #L_line = np.array([[l_rho*math.cos(self.theta),l_rho*math.sin(self.theta),-l_rho*self.rho, (1+self.rho*self.rho)*math.sin(self.theta), -(1+self.rho*self.rho)*math.cos(self.theta),0],\
                #[l_theta*math.cos(self.theta),l_theta*math.sin(self.theta),-l_theta * self.rho, -self.rho*math.cos(self.theta),-self.rho*math.sin(self.theta),-1]])

            L_line = np.array([[l_rho*math.cos(self.theta),l_rho*math.sin(self.theta),-l_rho*self.rho, (1+self.rho*self.rho)*math.sin(self.theta), -(1+self.rho*self.rho)*math.cos(self.theta),0],\
                [l_theta*math.cos(self.theta),l_theta*math.sin(self.theta),-l_theta * self.rho, -self.rho*math.cos(self.theta),-self.rho*math.sin(self.theta),-1]])

            if goal.mod == 0:
                self.cmd.yaw = 0
                self.cmd.pitch = 0
                self.cmd.roll = 0
                self.cmd.throttle = 0

            elif goal.mod == 1:

                self.v_y_s = 20

                self.flag = 0

            elif goal.mod == 2:

                self.v_y_s = -20

                self.flag = 0

            else:
                print("---Not Valid---")

            J_s_pseudo = np.linalg.pinv(np.vstack((L_line @ J_,[[0,0,1,0],[0,1,0,0]])))

            #s = np.array([[self.rho],[self.theta],[0]])
            #s = np.array([[self.rho],[self.theta]])
            #s = np.array([[self.rho],[self.z],[self.theta]])
            s_star = np.array([[self.rho_s],[self.theta_s],[self.z_s],[self.v_y_s]])
            s = np.array([[self.rho],[self.theta],[self.z],[self.v_y]])

            self.U = -self.lam * J_s_pseudo @ (s-s_star)

            if y_line<0:
                self.cmd.roll = -abs(self.U[0,0])    # movimento sulle x

            if y_line>0:
                self.cmd.roll = abs(self.U[0,0])    # movimento sulle x

            
            #self.cmd.roll = self.U[0,0]
            #self.cmd.pitch = self.v_y_s#self.U[1,0]
            self.cmd.pitch = self.U[1,0]
            self.cmd.throttle = self.U[2,0]
            self.cmd.yaw = self.U[3,0]     # movimento sullo yaw
            #self.cmd.throttle = self.P_z*(self.z_s - self.z)

            
            if(abs(self.cmd.throttle)>4): # MAX throttle DJI= 4m/s
                self.cmd.throttle=4*(abs(self.cmd.throttle)/self.cmd.throttle)

            if(abs(self.cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
                self.cmd.yaw=30*(abs(self.self.cmd.yaw)/self.cmd.yaw)
            
            if(abs(self.cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
                self.cmd.roll=5*(abs(self.cmd.roll)/self.cmd.roll)

            if(abs(self.cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
                self.cmd.pitch=5*(abs(self.cmd.pitch)/self.cmd.pitch)
            

            if(self.rail_detected==42): # It means rails not detected, so keep the drone still
                self.cmd.yaw = 0
                self.cmd.pitch = 0
                self.cmd.roll = 0
                self.cmd.throttle = 0


            if goal.mod == 0:
                if self.flag == 0:
                    pub_cmd_vel.publish(self.cmd)
                    
                else: 
                    self.flag = 1

            elif goal.mod == 1 or goal.mod == 2:
                pub_cmd_vel.publish(self.cmd)

            else: 
                print("---Not Valid---")

            #print("\nrail detected: ",(self.rail_detected!=42)," x:",self.x,", y:",self.y,", angle:",self.theta,", ground distance:",self.z)
            print("---")
            print("commands: ")
            print("yaw     : ", self.cmd.yaw)
            print("pitch   : ", self.cmd.pitch)
            print("roll    : ", self.cmd.roll)
            print("throttle: ", self.cmd.throttle)

            print("---")
            print("Data    :")
            print("y_line  :",y_line)
            print("z       :",self.z)
            print("theta   :",self.theta)
            print("v_y     :",self.v_y)

            self._feedback.x = self.x_current
            self._feedback.y = self.y_current
            self._feedback.z = self.z

            self.server.publish_feedback(self._feedback)
            rate.sleep()

            i = i+1

        self.cmd.roll     = 0.0
        self.cmd.pitch    = 0.0
        self.cmd.yaw      = 0.0
        self.cmd.throttle = 0.0

        pub_cmd_vel.publish(self.cmd)

        if self.x_current < (self.x_s+delta) and self.x_current > (self.x_s-delta) and self.y_current < (self.y_s+delta) and self.y_current > (self.y_s-delta):
            self._result.arrived = 1
            self.server.set_succeeded(self._result)

        else:
            self._result.arrived = 0
            self.server.set_aborted(self._result)




if __name__ == '__main__':

    rospy.init_node('follow_line_server')

    server = FollowLineServer()

    sub_1 = rospy.Subscriber("/localization", Pose, server.callback_loc)
    sub_2 = rospy.Subscriber("/ground_distance", Float32, server.callback_ground)
    sub_3 = rospy.Subscriber('/quadrotor_pose', Pose, server.get_rotation)
    rospy.spin()