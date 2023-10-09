#! /usr/bin/env python3

import math
import roslib
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

from insp_rail_pkg.msg import *

from   std_msgs.msg       import Float32,Int16
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from my_custom_interfaces.msg import Drone_cmd


# Pubblico comando in velocita con custom message sul topic command
pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel



P_gain_yaw=0.2
D_gain_yaw=0.5
I_gain_yaw=0#0.0001

P_gain_throttle=0.5
D_gain_throttle=0.1
I_gain_throttle=0#0.0001

P_gain_pitch=P_gain_roll=0.002
D_gain_pitch=D_gain_roll=0.001
I_gain_pitch=I_gain_roll=0#0.0001


class FollowLineServer:


    _feedback = FollowLineFeedback() 


    _result = FollowLineResult()

    def __init__(self):

        self.server = actionlib.SimpleActionServer('follow_line', FollowLineAction, self.execute, False)

        # GLOBAL VARIABLES
        self.x=float(0)
        self.y=float(0)
        self.angle=float(0)
        self.rail_detected=float(0)

        self.x_current = 0.0
        self.y_current = 0.0
        self.ground_distance=float(0)
        self.yaw = 0.0

        self.x_s = float(0)
        self.y_s = float(0)
        self.z_s = float(7)

        self.mod=0
        self.flag=0

        self.ei_z   = 0.0
        self.ei_y   = 0.0

        self.ed_z   = 0.0
        self.ed_y   = 0.0


        self.e_old_z   = 0.0
        self.e_old_y   = 0.0

        self.e_y   = 0.0
        self.e_z   = 0.0



        self.cmd =Drone_cmd()

        self.server.start()

    def update_olds(self):

        self.e_old_y   = self.e_y
        self.e_old_z   = self.e_z

    def update_integrals(self):

        self.ei_z   =  self.ei_z + self.e_z
        self.ei_y   =  self.ei_y + self.e_y


    def update_derivates(self):

        self.ed_y = self.e_y - self.e_old_y
        self.ed_z = self.e_z - self.e_old_z

    def update_errors(self,y_line):

        self.e_y = y_line
        self.e_z = self.z_s - self.ground_distance



    def get_rotation(self,data1):

        self.x_current  = data1.position.x
        self.y_current  = data1.position.y

        quat_x   = data1.orientation.x 
        quat_y   = data1.orientation.y
        quat_z   = data1.orientation.z
        quat_w   = data1.orientation.w

        orientation_list = [quat_x, quat_y, quat_z, quat_w]
        
        r, p, self.yaw = euler_from_quaternion (orientation_list)

    def callback_loc(self, pose):
        
        self.x = pose.position.x
        self.y = pose.position.y
        self.angle = pose.orientation.z 
        self.rail_detected = pose.orientation.w

    def callback_ground(self, distance):

        self.ground_distance = distance.data

    def execute(self, goal):


        global pub_cmd_vel
        global P_gain_yaw,P_gain_roll,P_gain_pitch,P_gain_throttle
        global I_gain_yaw,I_gain_roll,I_gain_pitch,I_gain_throttle
        global D_gain_yaw,D_gain_roll,D_gain_pitch,D_gain_throttle

        rate = rospy.Rate(20) # 20hz 

        delta = .3
        alpha = 30 # deg


        self.x_s = goal.x
        self.y_s = goal.y

        i = 0
        while not rospy.is_shutdown():

            self.x_s = goal.x
            self.y_s = goal.y
            print("self.x_s:",self.x_s)
            print("self.y_s:",self.y_s)
            if i>3 and self.x_current < (self.x_s+delta) and self.x_current > (self.x_s-delta) and self.y_current < (self.y_s+delta) and self.y_current > (self.y_s-delta):

                print(self.x_current,">",self.x_s+delta," ",self.x_current,"<",self.x_s-delta)
                print(self.y_current,">",self.y_s+delta," ",self.y_current,"<",self.y_s-delta)

                break

            if self.server.is_preempt_requested():

                rospy.loginfo('Preempted')

                #self.server.set_preempted()

                break
            
            rad_angle=math.radians(self.angle)

            if(self.angle==0):
                x_line=0
                y_line=self.y

            else:
                m=math.tan(-math.pi/2-rad_angle)
                x_line=m*(m*self.x-self.y)/(1+m*m)
                y_line=-(m*self.x-self.y)/(1+m*m)

            if(y_line>0):
                y_perp = math.sqrt(x_line*x_line+y_line*y_line)
            else:
                y_perp = -math.sqrt(x_line*x_line+y_line*y_line)



            self.update_errors(y_line)

            self.update_derivates()

            if (self.e_z+self.e_y) > 1.5:
                self.update_integrals()


            if goal.mod == 0:
                self.cmd.yaw = 0
                self.cmd.pitch = 0
                self.cmd.roll = 0
                self.cmd.throttle = 0



            elif goal.mod == 1:

                self.cmd.roll = (P_gain_pitch * self.e_y + D_gain_pitch * self.ed_y + I_gain_pitch * self.ei_y)*math.cos(math.radians(alpha))

                if (self.z_s-delta)< self.ground_distance and (self.z_s+delta)> self.ground_distance:
                    self.cmd.throttle = (P_gain_pitch * self.e_y + D_gain_pitch * self.ed_y + I_gain_pitch * self.ei_y)*math.sin(math.radians(alpha))

                else: 
                    self.cmd.throttle = P_gain_throttle*self.e_z 


                self.cmd.pitch = 0.2#max(0.2, abs(y_line)/50)
                self.flag = 0



            elif goal.mod == 2:

                self.cmd.roll = (P_gain_pitch * self.e_y + D_gain_pitch * self.ed_y + I_gain_pitch * self.ei_y)*math.cos(math.radians(alpha))

                if (self.z_s-delta)< self.ground_distance and (self.z_s+delta)> self.ground_distance:
                    self.cmd.throttle = (P_gain_pitch * self.e_y + D_gain_pitch * self.ed_y + I_gain_pitch * self.ei_y)*math.sin(math.radians(alpha))

                else: 
                    self.cmd.throttle = P_gain_throttle*self.e_z 


                self.cmd.pitch = -0.2#max(0.2, abs(y_line)/50)
                self.flag = 0

            else:
                print("---Not Valid---")



            
            if(abs(self.cmd.throttle)>4): # MAX throttle DJI= 4m/s
                self.cmd.throttle=4*(abs(self.cmd.throttle)/self.cmd.throttle)
            
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

            print("\nrail detected: ",(self.rail_detected!=42)," x:",self.x,", y:",self.y,", angle:",self.angle,", ground distance:",self.ground_distance)
            print("commands: ")
            print("yaw: ", self.cmd.yaw)
            print("pitch: ",self.cmd.pitch)
            print("roll: ", self.cmd.roll)
            print("throttle: ", self.cmd.throttle)
            print("y_line: ",y_line)
            print("mod: ",goal.mod)
            print("---")
            print("x:",self.x_current)
            print("y:",self.y_current)
            print("z:",self.ground_distance)

            self._feedback.x = self.x_current
            self._feedback.y = self.y_current
            self._feedback.z = self.ground_distance

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