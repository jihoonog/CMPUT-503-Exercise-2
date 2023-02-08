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
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped,LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern

from std_msgs.msg import Header, Float32, String, Float64MultiArray 


VERBOSE = 1

class StateControlNode(DTROS):

    def __init__(self, node_name):
        """Handles the state of the robot
        """

        # Initialize the DTROS parent class
        super(StateControlNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        if os.environ["VEHICLE_NAME"] is not None:
            self.veh_name = os.environ["VEHICLE_NAME"]
        else:
            self.veh_name = "csc22945"

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
        


    def pub_command(self, comm, params):
        command = comm + ':'+ params
        self.pub_motor_commands.publish(command)
        self._motor_command_executing = True

    def color_pattern(self,mode):
        msg = LEDPattern()
        if mode == 1:
            msg.color_list = ['red', 'red', 'red', 'red', 'red']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==2:
            msg.color_list = ['blue', 'blue', 'blue', 'blue', 'blue']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==3:
            msg.color_list = ['green', 'green', 'green', 'green', 'green']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==4:
            msg.color_list = ['purple', 'purple', 'purple', 'purple', 'purple']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==5:
            msg.color_list = ['blue', 'red', 'green', 'red', 'red']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]

        return msg

        

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
            rospy.sleep(0.5)

    def run_logic(self):
        serve_name = f"{self.veh_name}/led_emitter_node/set_custom_pattern"
        rospy.wait_for_service(serve_name)
        print("Starting logic")
        emitter_service = rospy.ServiceProxy(serve_name, SetCustomLEDPattern,persistent=True)
        
        
        emitter_service(self.color_pattern(1))
        self.block()
        # stage 1, sleep 5 secs
        rospy.sleep(5)
        # stage 2
        emitter_service(self.color_pattern(2))
        self.block()
        self.pub_command("right","90")
        self.block()
        self.pub_command("forward","1.1")
        self.block()
        self.pub_command("left","90")
        self.block()
        self.pub_command("forward","1.1")
        self.block()
        self.pub_command("left", "90")
        self.block()
        self.pub_command("forward","1.1")
        self.block()

        # Change color pattern to pattern 1 and sleep for 5 secs.
        emitter_service(self.color_pattern(1))
        rospy.sleep(5)

        # Stage 3, move back to the initial location and orientation.
        emitter_service(self.color_pattern(3))
        self.block()
        self.pub_command("left", "90")
        self.block()
        self.pub_command("forward","1.1")
        self.block()
        self.pub_command("right","180")
        self.block()

        # Change color pattern to pattern 1 and sleep for 5 secs.
        emitter_service(self.color_pattern(1))
        rospy.sleep(5)

        # Stage 4, clockwise circular movement, color pattern 4.
        emitter_service(self.color_pattern(4))
        self.block()
        self.pub_command("forward", "0.5")
        self.block()
        self.pub_command("arc_right","380:0.47")
        self.block()
        rospy.sleep(5)
    
    def on_shutdown(self):
        self.pub_command("shutdown", "shutdown")

if __name__ == '__main__':
    node = StateControlNode(node_name='state_control_node')
    # Keep it spinning to keep the node alive
    # Main loop
    print("Starting program")
    node.run_logic()
    print("Shutting down")
    rospy.signal_shutdown("Done")