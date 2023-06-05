#! /usr/bin/env python3

import roslib
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

from insp_rail_pkg.msg import *

from   std_msgs.msg       import Float32
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from my_custom_interfaces.msg import Drone_cmd


P_gain_yaw=0.2
D_gain_yaw=0.05
I_gain_yaw=0#0.0001

P_gain_throttle=0.5
D_gain_throttle=0.01
I_gain_throttle=0#0.0001

P_gain_pitch=P_gain_roll=0.2
D_gain_pitch=D_gain_roll=0.02
I_gain_pitch=I_gain_roll=0#0.0001


# pubblico comando in velocita con custom message sul topic coomand
pub_cmd_vel=rospy.Publisher("/command", Drone_cmd, queue_size=1) #maybe is better to use cmd_vel



class GoPoseServer:


  _feedback = GoPoseFeedback() 

  #
  _result = GoPoseResult()

  def __init__(self):

    self.server = actionlib.SimpleActionServer('go_to_goal', GoPoseAction, self.execute, False)

    self.x_current = 0.0
    self.y_current = 0.0
    self.z_current = 0.0
    self.yaw   = 0.0


    self.x_s = 0.0
    self.y_s = 0.0
    self.z_s = 0.0
    self.ang_s = 0.0

    self.ei_ang = 0.0
    self.ei_z   = 0.0
    self.ei_y   = 0.0
    self.ei_x   = 0.0

    self.ed_ang = 0.0
    self.ed_z   = 0.0
    self.ed_y   = 0.0
    self.ed_x   = 0.0

    self.e_old_ang = 0.0
    self.e_old_z   = 0.0
    self.e_old_y   = 0.0
    self.e_old_x   = 0.0

    self.e_x   = 0.0
    self.e_y   = 0.0
    self.e_z   = 0.0
    self.e_ang = 0.0

    self.cmd =Drone_cmd()

    self.server.start()

  def update_olds(self):

    self.e_old_x   = self.e_x 
    self.e_old_y   = self.e_y
    self.e_old_z   = self.e_z
    self.e_old_ang = self.e_ang


  def update_integrals(self):

    self.ei_ang =  self.ei_ang + self.e_ang
    self.ei_z   =  self.ei_z + self.e_z
    self.ei_y   =  self.ei_y + self.e_y
    self.ei_x   =  self.ei_x + self.e_x 

  def update_derivates(self):

    self.ed_x = (self.e_x - self.e_old_x)/(1/20)
    self.ed_y = (self.e_y - self.e_old_y)/(1/20)
    self.ed_z = (self.e_z - self.e_old_z)/(1/20)
    self.ed_ang = (self.e_ang - self.e_old_ang)/(1/20)

  def update_errors(self):

    self.e_x = self.x_s - self.x_current
    self.e_y = self.y_s - self.y_current
    self.e_z = self.z_s - self.z_current
    self.e_ang = self.ang_s - self.yaw


  def get_rotation(self,data1):

    self.x_current  = data1.position.x
    self.y_current  = data1.position.y

    quat_x   = data1.orientation.x 
    quat_y   = data1.orientation.y
    quat_z   = data1.orientation.z
    quat_w   = data1.orientation.w

    orientation_list = [quat_x, quat_y, quat_z, quat_w]
    
    r, p, self.yaw = euler_from_quaternion (orientation_list)

  def callback_ground(self, distance):

    self.z_current = distance.data




  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here

    print("execute")
    global pub_cmd_vel
    global P_gain_yaw,P_gain_roll,P_gain_pitch,P_gain_throttle
    global I_gain_yaw,I_gain_roll,I_gain_pitch,I_gain_throttle
    global D_gain_yaw,D_gain_roll,D_gain_pitch,D_gain_throttle

    self.x_s = goal.x
    self.y_s = goal.y
    self.z_s = goal.z

    delta = .1

    i = 0
    while not rospy.is_shutdown():

      if i>3 and self.x_current < (self.x_s+delta) and self.x_current > (self.x_s-delta) and self.y_current < (self.y_s+delta) and self.y_current > (self.y_s-delta):
        
        print(self.x_current,">",self.x_s+delta," ",self.x_current,"<",self.x_s-delta)
        print(self.y_current,">",self.y_s+delta," ",self.y_current,"<",self.y_s-delta)

        break

      #print("In the loop")
      # check that preempt has not been requested by the client
      if self.server.is_preempt_requested():
        rospy.loginfo('Preempted')
        #self.server.set_preempted()
        break

      rate = 20
      ra = rospy.Rate(rate)

      self._feedback.x = self.x_current
      self._feedback.y = self.y_current
      self._feedback.z = self.z_current

      self.server.publish_feedback(self._feedback)

      self.x_s = goal.x
      self.y_s = goal.y
      self.z_s = goal.z

      self.update_errors()

      self.update_derivates()
      
      print(self.x_current)
      print(self.y_current)
      print(self.z_current)

      if (self.e_ang+self.e_z+self.e_y+self.e_x) > 1.5:
        self.update_integrals()

      self.cmd.roll = P_gain_pitch * self.e_y + D_gain_pitch * self.ed_y + I_gain_pitch * self.ei_y

      if(abs(self.cmd.roll)>5): # MAX roll/pitch DJI= 15m/s 
        self.cmd.roll=5*(abs(self.cmd.roll)/self.cmd.roll)

      self.cmd.pitch = -(P_gain_roll * self.e_x + D_gain_roll * self.ed_x + I_gain_roll * self.ei_x)

      if(abs(self.cmd.pitch)>5): # MAX roll/pitch DJI= 15m/s 
        self.cmd.pitch=5*(abs(self.cmd.pitch)/self.cmd.pitch)

      self.cmd.throttle = P_gain_throttle * self.e_z + D_gain_throttle * self.ed_z + I_gain_throttle * self.ei_z

      if(abs(self.cmd.throttle)>4): # MAX throttle DJI= 4m/s
        self.cmd.throttle=4*(abs(self.cmd.throttle)/self.cmd.throttle)

      self.cmd.yaw = P_gain_yaw * self.e_ang + D_gain_yaw * self.ed_ang + I_gain_yaw * self.ei_ang

      if(abs(self.cmd.yaw)>30): # MAX yaw DJI= 100 degree/s 
        self.cmd.yaw=30*(abs(self.cmd.yaw)/self.cmd.yaw)


      self.update_olds()

      pub_cmd_vel.publish(self.cmd)

      ra.sleep()

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
  rospy.init_node('go_to_goal_server')

  server = GoPoseServer()

  # aggiunto io per prendere la posizione del drone dalla simulazione 
  sub = rospy.Subscriber('/quadrotor_pose', Pose, server.get_rotation)
  sub_2 = rospy.Subscriber("/ground_distance", Float32, server.callback_ground)

  rospy.spin()