#! /usr/bin/env python3

import roslib
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

import datetime

from insp_panels_pkg.msg import *

from   std_msgs.msg       import Float32
from   geometry_msgs.msg  import Pose


# pubblico comando in velocita con custom message sul topic coomand
pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel



class GoPoseServer:


  _feedback = GoPoseFeedback() 

  #
  _result = GoPoseResult()

  def __init__(self):

    self.server = actionlib.SimpleActionServer('go_to_goal', GoPoseAction, self.execute, False)

    self.cmd =Drone_cmd()

    self.server.start()

    self.T = 0.0
    
    self.delta_x = 0.0
    self.delta_y = 0.0

  def poly_line_coef(self, s_i,s_f,v_i,v_f,a_i,a_f,v_max):

    cv = 1.875
    self.T = (cv/v_max)* s_f
    a_0= s_i
    a_1= v_i
    a_2= 1/2 * a_i
    a_3= 1/2 * 1/(pow(self.T,3))*(20*s_f-(8*v_f+12*v_i)*self.T-(3*a_i-a_f)*pow(self.T,2))
    a_4= 1/2 * 1/(pow(self.T,4))*(-30*s_f+(14*v_f+16*v_i)*self.T+(3*a_f-2*a_i)*pow(self.T,2))
    a_5= 1/2 * 1/(pow(self.T,5))*(12*s_f-6*(v_f+v_i)*self.T+(a_f-a_i)*pow(self.T,2))
    return a_0,a_1,a_2,a_3,a_4,a_5

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here

    print("execute")

    self.delta_x = goal.x
    self.delta_y = goal.y
    
    rate = 20
    ra = rospy.Rate(rate)
    

    a_0_x,a_1_x,a_2_x,a_3_x,a_4_x,a_5_x = self.poly_line_coef(0,self.delta_x,0,0,0,0,0.2)
    a_0_y,a_1_y,a_2_y,a_3_y,a_4_y,a_5_y = self.poly_line_coef(0,self.delta_y,0,0,0,0,0.2)

    check = datetime.datetime.now()+ datetime.timedelta(seconds = self.T)
    t_0 = datetime.datetime.now()

    print
    while not rospy.is_shutdown():

      #print("In the loop")
      # check that preempt has not been requested by the client
      if self.server.is_preempt_requested():
        rospy.loginfo('Preempted')
        #self.server.set_preempted()
        break

      self._feedback.x = self.delta_x
      self._feedback.y = self.delta_y
      self._feedback.z = 7.0

      t = datetime.datetime.now()

      self.server.publish_feedback(self._feedback)
      self.cmd.roll = a_1_y+ a_2_y*2*(t-t_0).total_seconds()+ a_3_y*3*pow((t-t_0).total_seconds(),2)+ a_4_y*4*pow((t-t_0).total_seconds(),3)+ a_5_y*5*pow((t-t_0).total_seconds(),4)

      if(abs(self.cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
        self.cmd.roll=5*(abs(self.cmd.roll)/self.cmd.roll)

      self.cmd.pitch = a_1_x+ a_2_x*2*(t-t_0).total_seconds()+ a_3_x*3*pow((t-t_0).total_seconds(),2)+ a_4_x*4*pow((t-t_0).total_seconds(),3)+ a_5_x*5*pow((t-t_0).total_seconds(),4)

      if(abs(self.cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
        self.cmd.pitch=5*(abs(self.cmd.pitch)/self.cmd.pitch)
      
      self.cmd.yaw = 0.0
      self.cmd.throttle = 0.0

      pub_cmd_vel.publish(self.cmd)

      if datetime.datetime.now()>check:
        break

      ra.sleep()

    self.cmd.roll     = 0.0
    self.cmd.pitch    = 0.0
    self.cmd.yaw      = 0.0
    self.cmd.throttle = 0.0

    pub_cmd_vel.publish(self.cmd)

    self.server.set_succeeded(self._result)




if __name__ == '__main__':
  rospy.init_node('go_to_goal_server')

  server = GoPoseServer()

  rospy.spin()