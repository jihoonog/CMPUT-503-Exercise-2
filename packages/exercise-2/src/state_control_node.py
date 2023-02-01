#!/usr/bin/env python3

"""
This will maintain the state of the robot for exercise 2. 
It will also handle the state transitions and when appropriate give control commands to the motor control node based on the feedback.
"""

import numpy as np
import os
import math
import time
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, String, Float64MultiArray




class StateControlNode(DTROS):

    def __init__(self, node_name):
        """Handles the state of the robot
        """

        # Initialize the DTROS parent class
        super(StateControlNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # if os.environ["VEHICLE_NAME"] is not None:
        #     self.veh_name = os.environ["VEHICLE_NAME"]
        # else:
        # This might need to be changed
        self.veh_name = "csc22935"

        self.rate = rospy.Rate(10)

        # State
        self._state = 1 # The initial state

        # Subscribers
        ## Distance the wheels have traveled
        self.sub_wheel_dist_traveled = rospy.Subscriber(f'/motor_control_node/wheel_dist_traveled', Float64MultiArray, self.cb_get_wheel_dists)
        
        
        # Internal encoder state
        self.left_wheel_ticks = 0
        self.right_wheel_ticks = 0
        self.left_wheel_offset = 0
        self.right_wheel_offset = 0
        self.initial_left_tick = True
        self.initial_right_tick = True
        self.left_dist = 0
        self.right_dist = 0

        # Publishers
        self.pub_motor_commands = rospy.Publisher(f'/state_control_node/command', String, queue_size=1)
        # self.pub_integrated_distance_left = rospy.Publisher(...)
        # self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")


    def cb_get_wheel_dists(self, msg):
        """
        msg - the distances the wheels have traveled
        """
        left_wheel = msg.data[0]
        right_wheel = msg.data[1]

    def pub_command(self, comm, params):
        command = comm + ':'+ params
        self.pub_motor_commands.publish(command)

    def run_logic(self):
        self.pub_command("forward","1250")
        time.sleep(20)
        self.pub_command("right","90")
        time.sleep(20)
        self.pub_command("left","90")
        time.sleep(20)
    
    def on_shutdown(self):
        pass

if __name__ == '__main__':
    node = StateControlNode(node_name='state_control_node')
    # Keep it spinning to keep the node alive
    # Main loop
    time.sleep(4)
    while not rospy.is_shutdown():
        node.run_logic()
    rospy.spin()
    rospy.loginfo("state control node is up and running...")