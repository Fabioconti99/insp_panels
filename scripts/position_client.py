#! /usr/bin/env python3

import rospy
import numpy as np

# Brings in the SimpleActionClient
import actionlib

from insp_rail_pkg.msg import *

from   std_msgs.msg       import Float64,Int16
from   geometry_msgs.msg  import Pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

from my_custom_interfaces.msg import Drone_cmd


# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
client = actionlib.SimpleActionClient('go_to_goal', GoPoseAction)

stop_ = 0
def cancel_callback(data1):
    global client, stop_

    if data1.data==-1:
        client.cancel_all_goals()
        stop_ = 1

def pose_client():
    global client, stop_

    #wp = np.array([[19.525,20.625],[9.525,20.6],[-11.05,15.375],[-19.550,15.375],[-19.975,0.350],[-10.975,0.350],[9.6,5.6],[19.1,5.6],[16.775,-9.575],[9.275,-9.575],[-11.1,-14.725],[-22.3,-14.825]])
    wp = np.array([[20,22],[-20,22],[-20,18.5],[20,18.5],[20,15],[-20,15],[-20,10],[20,10],[20,6.5],[-20,6.5],[-20,3],[20,3],[20,-2.5],[-20,-2.5],[-20,-6],[20,-6],[20,-9.5],[-20,-9.5],[-20,-15],[20,-15],[20,-18.5],[-20,-18.5],[-20,-22],[20,-22]])

    print("pose_client")
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = GoPoseGoal()

    i = 0
    rate = 20000
    ra = rospy.Rate(rate)
    while not rospy.is_shutdown():

        try:
            ra.sleep()

            print("x_goal: ",wp[i,0])
            print("y_goal: ",wp[i,1])
            print("z_goal: ",5.0)

            x_goal = wp[i,0]

            y_goal = wp[i,1]

            z_goal = 7.0

            goal.x = float(x_goal)
            goal.y = float(y_goal)
            goal.z = float(z_goal)

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

    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pose_client_py')

        sub = rospy.Subscriber('/cancel_UI', Int16, cancel_callback)
        result = pose_client()

        print(result.arrived)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)