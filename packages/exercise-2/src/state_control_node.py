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
        self.veh_name = "csc22945"

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
        



    def cb_get_wheel_dists(self, msg):
        """
        msg - the distances the wheels have traveled
        """
        left_wheel = msg.data[0]
        right_wheel = msg.data[1]

    def pub_command(self, comm, params):
        command = comm + ':'+ params
        self.pub_motor_commands.publish(command)

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


    def run_logic(self):
        left =0.2
        right=-0.2

        motor_cmd = WheelsCmdStamped()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.vel_left = left
        motor_cmd.vel_right = right   
        if SIM:
            self.left_wheel_ticks += int(left * 5 + 1) 
            self.right_wheel_ticks += int(right * 5 + 1)

            self.calculate_dist_traveled()
        else:
            self.pub_motor_commands.publish(motor_cmd)
        

    
    def on_shutdown(self):
        pass

if __name__ == '__main__':
    node = StateControlNode(node_name='state_control_node')
    # Keep it spinning to keep the node alive
    # Main loop
    time.sleep(5)
    while not rospy.is_shutdown():
        node.run_logic()
    rospy.spin()
    rospy.loginfo("state control node is up and running...")