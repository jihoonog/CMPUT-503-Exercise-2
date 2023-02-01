#!/usr/bin/env python3
import numpy as np
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # if os.environ["VEHICLE_NAME"] is not None:
        #     self.veh_name = os.environ["VEHICLE_NAME"]
        # else:
        self.veh_name = "csc22935"

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='right')
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)
        
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
        # self.pub_integrated_distance_left = rospy.Publisher(...)
        # self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")

    def calculate_dist_traveled(self):
        """ Calculate the distance travelled by each wheel.
        """
        left_dist = (self.left_wheel_ticks - self.left_wheel_offset) * self._radius * 2 * math.pi / 135
        right_dist = (self.right_wheel_ticks - self.right_wheel_offset) * self._radius * 2 * math.pi / 135
        if abs(self.left_dist - left_dist) > 0.001:
            self.left_dist = left_dist
            print("Left distance: ", left_dist)
        if abs(self.right_dist - right_dist) > 0.001 :
            self.right_dist = right_dist
            print("Right distance: ", right_dist)


    def cb_encoder_data(self, wheel, msg):
        """
        msg - which wheel is the ros topic associated with
        wheel - the wheel data
        """
        """ Update encoder distance information from ticks.
        """
        if self.initial_left_tick and msg == 'left':
            self.left_wheel_offset = wheel.data
            self.initial_left_tick = False
    
        if self.initial_right_tick and msg == 'right':
            self.right_wheel_offset = wheel.data
            self.initial_right_tick = False
    
        if msg == 'left' and wheel.data != self.left_wheel_ticks:
            self.left_wheel_ticks = wheel.data

        if msg == 'right' and wheel.data != self.right_wheel_ticks:
            self.right_wheel_ticks = wheel.data

        self.calculate_dist_traveled()
        

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        print("Executed command: ", msg)

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")