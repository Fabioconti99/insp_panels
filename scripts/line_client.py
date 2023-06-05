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
client = actionlib.SimpleActionClient('follow_line', FollowLineAction)

stop_ = 0
def cancel_callback(data1):
    global client, stop_

    if data1.data==-1:
        client.cancel_all_goals()
        stop_ = 1

def line_client():
    global client, stop_

    wp = np.array([[19.525,20.625],[9.525,20.6]])

    print("pose_client")
    # Creates the SimpleActionClient, passing the type of the action

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FollowLineGoal()

    i = 0
    rate = 20000
    ra = rospy.Rate(rate)
    while not rospy.is_shutdown():

        try:
            ra.sleep()

            print("wp[i+1,0]",wp[i+1,0])
            print("wp[i,0]",wp[i,0])

            if float(wp[i+1,0]) > float(wp[i,0]):

                goal.mod = int(2)


            else:
                
                goal.mod = int(1)

            goal.x = wp[i+1,0]
            goal.y = wp[i+1,1]

            # Sends the goal to the action server.
            client.send_goal(goal)

            # Waits for the server to finish performing the action.
            client.wait_for_result()

            i=i+1

            if stop_ == 1:

                break

        except:

            break

    if stop_!=1:
        print("Arrived")
    else:
        print("Cancelled goals")
    # Prints out the result of executing the action

    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('line_client_py')

        sub = rospy.Subscriber('/cancel_UI', Int16, cancel_callback)
        result = line_client()

        print(result.arrived)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
