#!/usr/bin/env python3

"""
This will control the motors for the duckiebots for exercise 2.
It will publish velocity commands to the wheels_driver_node.
It will also publish the distance traveled by each wheel.
It's also subscribed to the 
"""

import numpy as np
import os
import math
import rospy


from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, String, Float64MultiArray 


# Change this before executing
VERBOSE = 0
SIM = False


class MotorControlNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(MotorControlNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        # if os.environ["VEHICLE_NAME"] is not None:
        #     self.veh_name = os.environ["VEHICLE_NAME"]
        # else:
        # This might need to be changed
        self.veh_name = "csc22945"

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribers to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='right')
        # Subscribers to the state control node
        self.sub_state_control_comm = rospy.Subscriber(f'/state_control_node/command', String, self.cb_state_control_comm)
        # self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='left')
        # self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='right')
        # self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)

        # Internal encoder state
        self.left_wheel_ticks = 0
        self.right_wheel_ticks = 0
        self.left_wheel_offset = 0
        self.right_wheel_offset = 0
        # These initial flags should be set to True if you want to reset distance traveled
        self.initial_left_tick = True
        self.initial_right_tick = True
        self.left_dist = 0
        self.right_dist = 0
        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        ## Publish what wheel commands have been executed
        self.pub_executed_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', String, queue_size = 1)
        ## Publish the distance traveled by each wheel
        self.pub_wheel_dist_traveled = rospy.Publisher(f'/motor_control_node/wheel_dist_traveled', Float64MultiArray, queue_size = 1)
        # self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/emergency_stop', WheelsCmdStamped, queue_size=1)

        self.log("Initialized")

    def calculate_dist_traveled(self):
        """ 
        Calculate the distance (in milimeters) traveled by each wheel.
        """
        left_dist = (self.left_wheel_ticks - self.left_wheel_offset) * self._radius * 2 * math.pi / 135
        right_dist = (self.right_wheel_ticks - self.right_wheel_offset) * self._radius * 2 * math.pi / 135
        
        # 0.001 is well within a single increment change of wheel displacement
        if abs(self.left_dist - left_dist) > 0.001:
            self.left_dist = left_dist
            if VERBOSE > 0:
                print("Left distance: ", left_dist)
        
        if abs(self.right_dist - right_dist) > 0.001:
            self.right_dist = right_dist
            if VERBOSE > 0:
                print("Right distance: ", right_dist)

    def reset_encoder_values(self):
        self.left_wheel_offset = self.left_wheel_ticks
        self.right_wheel_offset = self.right_wheel_ticks

    def cb_encoder_data(self, wheel, msg):
        """
        Update encoder distance information from ticks.
        
        msg - which wheel is the ros topic associated with
        wheel - the wheel data
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

    def cb_state_control_comm(self, msg):
        """
        Decode command from state control and call the right function with the given parameters
        """
        command = msg.data
        print("State Control Comm:", command)
        command_args =  command.split(":")
        if command_args[0].lower() == "forward":
            dist = float(command_args[1])
            self._move_forward(dist)
        elif command_args[0].lower() == "right":
            rotation = float(command_args[1])
            self._rotate_robot(-rotation)
        elif command_args[0].lower() == "left":
            rotation = float(command_args[1])
            self._rotate_robot(rotation)
        else:
            print("Not a valid command")

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        print("Executed command: ", msg)

    def _move_forward(self, dist):
        """
        Move the robot forward by dist amount
        """
        self.reset_encoder_values()
        self.calculate_dist_traveled()
        rate = rospy.Rate(100) # 100 Hz

        # maybe add some drift correction
        while self.left_dist < dist and self.right_dist < dist:
            # slow down so it doesn't over shoot
            if abs(dist - self.left_dist) < 100 or abs(dist - self.right_dist) < 100:
                self.command_motors(0.1, 0.1)
            else:
                self.command_motors(0.75, 0.75)
            #print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
            
            rate.sleep()

        self.command_motors(0.0, 0.0)
        print("Done moving forward:", dist)

    def _rotate_robot(self, degrees):
        """
        Rotate the robot by degrees degrees
        """
        self.reset_encoder_values()
        self.calculate_dist_traveled()
        rate = rospy.Rate(100) # 100 Hz
        # Figure out the reverse kinematics 
        L =  50.0 # in mm
        # calculating the arc length for each wheel
        target_arc_dist = abs(L * degrees * math.pi / 180.0) / 1000.0
        print("Target arc dist:", target_arc_dist)
        # left turn
        if degrees > 0:
            while self.left_dist > -target_arc_dist and self.right_dist < target_arc_dist:
                self.command_motors(-0.2, 0.2)
                print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
                rate.sleep()
        # right turn
        elif degrees < 0:
            while self.left_dist < target_arc_dist and self.right_dist > -target_arc_dist:
                self.command_motors(0.2, -0.2)
                print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
                rate.sleep()

        self.command_motors(0.0,0.0)
        print("Done rotation:", degrees, "degrees")


    def command_motors(self, left, right):
        """ This function is called periodically to send motor commands.
        """

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
        """Cleanup function."""
        motor_cmd = WheelsCmdStamped()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.vel_left = 0.0
        motor_cmd.vel_right = 0.0
        self.pub_motor_commands.publish(motor_cmd)

if __name__ == '__main__':
    node = MotorControlNode(node_name='motor_control_node')
    # Keep it spinning to keep the node alive
    # main loop
    rospy.spin()
