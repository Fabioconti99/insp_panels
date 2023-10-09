#! /usr/bin/env python3


import roslib
import time,math
roslib.load_manifest('insp_panels_pkg')
import rospy

from insp_panels_pkg.msg import *

from   std_msgs.msg       import Float32
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion


csv_name= "ext_alt_5.csv"
x = float(0)
y = float(0)
z = float(0)
yaw = float(0)

x_im = float(0)
y_im = float(0)
angle = float(0)

alt = float(0)
with open(csv_name,"a")as file:
    file.write("x_im,y_im,Theta_im,ts\n")

def callback_loc(data):
    global x,y,z,yaw
    x = data.position.x
    y = data.position.y
    z = data.position.z
    quat_x   = data.orientation.x 
    quat_y   = data.orientation.y
    quat_z   = data.orientation.z
    quat_w   = data.orientation.w

    orientation_list = [quat_x, quat_y, quat_z, quat_w]
        
    r, p, yaw = euler_from_quaternion (orientation_list)

def callback_im_feature(pose):
    global x_im,y_im,angle
    x = pose.position.x
    y = pose.position.y
    angle = pose.orientation.z 

    rad_angle=math.radians(angle)
    
    if(rad_angle==0):
        x_im=0
        y_im=y

    else:
        m=math.tan(rad_angle)
        x_im=m*(m*x-y)/(1+m*m)
        y_im=-(m*x-y)/(1+m*m)


def callback_alt(dat):
    global alt
    alt =  dat.data



if __name__ == '__main__':

    rospy.init_node('data_capture')
    sub= rospy.Subscriber("/localization", Pose, callback_im_feature)
    sub_2= rospy.Subscriber("/barometer_altitude", Float32, callback_alt)
    rate = rospy.Rate(20) # 20hz 
    while not rospy.is_shutdown():

        with open(csv_name,"a")as file:
            file.write("{},{},{},{},{}\n".format(str(x_im),str(y_im),str(angle),str(time.time()),str(alt)))
        print("Captured {},{},{},{},{}\n".format(str(x_im),str(y_im),str(angle),str(time.time()),str(alt)))
        rate.sleep()

    rospy.spin()