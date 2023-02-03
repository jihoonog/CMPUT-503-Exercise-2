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


VERBOSE = 1

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

        # State
        self._state = 1 # The initial state
        self._motor_command_executing = False # Flag to check if a command is being executed

        # Subscribers
        ## Distance the wheels have traveled
        self.sub_motor_control_comm_confirm = rospy.Subscriber(f'/wheels_driver_node/wheels_cmd_executed', String, self.cb_motor_control_confirm)
        
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

    def pub_command(self, comm, params):
        command = comm + ':'+ params
        self.pub_motor_commands.publish(command)
        self._motor_command_executing = True

    def cb_motor_control_confirm(self, msg):
        """
        Callback function that processes the confirmation message from the motor control node
        """
        confirm_msg = msg.data.split(":")
        if confirm_msg[0].lower() == "done":
            if VERBOSE > 0:
                print("Confirmation msg:", msg)
            self._motor_command_executing = False

    def block(self):
        while self._motor_command_executing:
            time.sleep(0.1)

    def run_logic(self):
        self.pub_command("right","103")
        self.block()
        self.pub_command("forward","1.05")
        self.block()
        self.pub_command("left","110")
        self.block()
        self.pub_command("forward","1.0")
        self.block()
        self.pub_command("left", "90")
        self.block()
        self.pub_command("forward","1.1")
        self.block()
        rospy.sleep(5)
        self.pub_command("left", "90")
        self.block()
        self.pub_command("forward","1.15")
        self.block()
        self.pub_command("right","180")
        self.block()
        rospy.sleep(5)
        self.pub_command("forward", "0.5")
        self.block()
        self.pub_command("arc_right","380:0.55")
        self.block()
        rospy.sleep(5)
    
    def on_shutdown(self):
        pass

if __name__ == '__main__':
    node = StateControlNode(node_name='state_control_node')
    # Keep it spinning to keep the node alive
    # Main loop
    print("Starting program")
    time.sleep(3)
    print("Starting logic")
    while not rospy.is_shutdown():
        node.run_logic()
    rospy.spin()
    rospy.loginfo("state control node is up and running...")