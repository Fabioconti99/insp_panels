#! /usr/bin/env python3

import math
import numpy as np
import roslib
roslib.load_manifest('insp_panels_pkg')
import rospy
import actionlib

from insp_rail_pkg.msg import *

from   std_msgs.msg       import Float32,Int16
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from my_custom_interfaces.msg import Drone_cmd


# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
client_1 = actionlib.SimpleActionClient('follow_line', FollowLineAction)
client_2 = actionlib.SimpleActionClient('go_to_goal', GoPoseAction)
wp = np.array([[-18,22],[-20,22],[-20,18.5],[20,18.5],[20,15],[-20,15],[-20,10],[20,10],[20,6.5],[-20,6.5],[-20,3],[20,3],[20,-2.5],[2.5,-2.5],[-2.5,-2.5],[-20,-2.5],[-20,-6],[-2.5,-6],[2.5,-6],[20,-6],[20,-9.5],[2.5,-9.5],[-2.5,-9.5],[-20,-9.5],[-20,-15],[-2.5,-15],[2.5,-15],[20,-15],[20,-18.5],[2.5,-18.5],[-2.5,-18.5],[-20,-18.5],[-20,-22],[-2.5,-22],[2.5,-22],[20,-22]])
#wp = np.array([[20,-2.5],[2.5,-2.5],[-2.5,-2.5],[-20,-2.5],[-20,-6],[-2.5,-6],[2.5,-6],[20,-6],[20,-9.5],[2.5,-9.5],[-2.5,-9.5],[-20,-9.5],[-20,-15],[-2.5,-15],[2.5,-15],[20,-15],[20,-18.5],[2.5,-18.5],[-2.5,-18.5],[-20,-18.5],[-20,-22],[-2.5,-22],[2.5,-22],[20,-22]])

wp[:,1]-= (7-(0.225+(1.5/2)*math.sin(math.radians(30))))*math.tan(math.radians(30))

stop_ = 0
def cancel_callback(data1): 
    global client_1, client_2, stop_

    if data1.data==-1:
        client_1.cancel_all_goals()
        client_2.cancel_all_goals()
        stop_ = 1



def pose_client(k):
    global client_2, stop_, wp

    print("pose_client")

    client_2.wait_for_server()

    # Creates a goal to send to the action server.
    goal = GoPoseGoal()

    ra.sleep()

    print("x_goal: ",wp[k,0])
    print("y_goal: ",wp[k,1])
    print("z_goal: ",5.0)

    x_goal = wp[k,0]

    y_goal = wp[k,1]

    z_goal = 7

    goal.x = float(x_goal)
    goal.y = float(y_goal)
    goal.z = float(z_goal)

    # Sends the goal to the action server.
    client_2.send_goal(goal)

    # Waits for the server to finish performing the action.
    client_2.wait_for_result()

    if stop_!=1:
        print("Arrived")
    else:
        print("Cancelled goals")
    # Prints out the result of executing the action

    return client_2.get_result()  # A FibonacciResult




def line_client(k):
    global client_1, stop_, wp

    print("pose_client")
    # Creates the SimpleActionClient, passing the type of the action

    # Waits until the action server has started up and started
    # listening for goals.
    client_1.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FollowLineGoal()

    print("wp[k,0]",wp[k,0])
    print("wp[k-1,0]",wp[k-1,0])

    if float(wp[k,0]) > float(wp[k-1,0]):

        goal.mod = int(2)

    else:
        
        goal.mod = int(1)

    goal.x = wp[k,0]
    goal.y = wp[k,1]

    # Sends the goal to the action server.
    client_1.send_goal(goal)

    # Waits for the server to finish performing the action.
    client_1.wait_for_result()

    if stop_!=1:
        print("Arrived")
    else:
        print("Cancelled goals")
    # Prints out the result of executing the action

    return client_1.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('line_client_py')

        sub = rospy.Subscriber('/cancel_UI', Int16, cancel_callback)

        rate = 20000
        ra = rospy.Rate(rate)

        for i in range(int(np.shape(wp)[0])):

            ra.sleep()

            if i%2 == 0:

                print("Pose controller")
                try:
                    result_1 = pose_client(i)

                except:
                    break

                if stop_==1:
                    print("process manually stopped")
                    break

                print(result_1.arrived)

            else:
                print("Line controller")    
                ra.sleep()

                try:
                    result_2 = line_client(i)

                except:
                    break

                if stop_==1:
                    print("process manually stopped")
                    break

                print(result_2.arrived)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
