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
from PID import PID


from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped
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
        if os.environ["VEHICLE_NAME"] is not None:
            self.veh_name = os.environ["VEHICLE_NAME"]
        else:
            self.veh_name = "csc22935"

        # Get static parameters
        # self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self._radius = 0.0318
        self.L = 0.05

        # Velocity settings
        self.forward_vel = 0.5
        self.rotation_vel = 0.25

        # Subscribers
        ## Subscribers to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data, callback_args='right')
        ## Subscribers to the state control node
        self.sub_state_control_comm = rospy.Subscriber(f'/state_control_node/command', String, self.cb_state_control_comm)

        # For PID controller
        self.Kp = 0.5
        self.Ki = 1.5
        self.Kd = 1
        self.sample_time = 0.25
        self.output_limits = (-0.05, 0.05)
        self.pid_controller = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd, 
            sample_time=self.sample_time, 
            output_limits=self.output_limits)

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
        self.prev_left_dist = 0
        self.prev_right_dist = 0

        # Pose Estimation
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = math.pi/2.0

        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        ## Publish what wheel commands have been executed
        self.pub_executed_commands = rospy.Publisher(f'/wheels_driver_node/wheels_cmd_executed', String, queue_size = 1)
        ## Publish the distance traveled by each wheel
        self.pub_wheel_dist_traveled = rospy.Publisher(f'/motor_control_node/wheel_dist_traveled', Float64MultiArray, queue_size = 1)
        # self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/emergency_stop', WheelsCmdStamped, queue_size=1)

        self.log("Initialized")

    def calculate_dist_traveled(self):
        """ 
        Calculate the distance (in meter) traveled by each wheel.
        """
        left_dist = (self.left_wheel_ticks - self.left_wheel_offset) * self._radius * 2 * math.pi / 135
        right_dist = (self.right_wheel_ticks - self.right_wheel_offset) * self._radius * 2 * math.pi / 135
        
        # 0.00001 is well within a single increment change of wheel displacement
        if abs(self.left_dist - left_dist) > 0.00001:
            self.left_dist = left_dist
            if VERBOSE > 0:
                print("Left distance: ", left_dist)
        
        if abs(self.right_dist - right_dist) > 0.00001:
            self.right_dist = right_dist
            if VERBOSE > 0:
                print("Right distance: ", right_dist)
    
    def calculate_pose(self):
        """
        This will calculate the pose based on the distance traveled by each wheel.
        """
        # Distance traveled by each wheel since last reset
        self.prev_left_dist = self.left_dist
        self.prev_right_dist = self.right_dist
        self.calculate_dist_traveled()
        delta_r = self.right_dist - self.prev_right_dist
        delta_l = self.left_dist - self.prev_left_dist
        # calculate the change in robot frame
        delta_rx = delta_l/2 + delta_r/2
        delta_ry = 0
        delta_rtheta = (delta_r/(2 * self.L)) - (delta_l/(2 * self.L))

        delta_ix = delta_rx * math.cos(self.pose_theta) - delta_ry * math.sin(self.pose_theta)
        delta_iy = delta_rx * math.sin(self.pose_theta) + delta_ry * math.cos(self.pose_theta)
        delta_itheta = delta_rtheta

        self.pose_x += delta_ix
        self.pose_y += delta_iy
        self.pose_theta += delta_itheta
        
        print("Pose: ", self.pose_x, self.pose_y, self.pose_theta * 180 / math.pi)

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
        elif command_args[0].lower() == "arc_left":
            rotation = float(command_args[1])
            radius = float(command_args[2])
            self._arc_robot(rotation, radius)
        elif command_args[0].lower() == "arc_right":
            rotation = float(command_args[1])
            radius = float(command_args[2])
            self._arc_robot(-rotation, radius)
        else:
            print("Not a valid command")


    def _move_forward(self, dist):
        """
        Move the robot forward by dist amount
        """
        self.reset_encoder_values()
        self.calculate_dist_traveled()
        self.trim_correction = -0.02
        rate = rospy.Rate(100)
        while self.left_dist < dist or self.right_dist < dist:
            # slow down so it doesn't over shoot
            dist_drift = self.left_dist - self.right_dist
            if VERBOSE > 0:
                print('dist_drift:', dist_drift)
            control = self.pid_controller(dist_drift)

            left_vel = self.forward_vel + control + self.trim_correction
            right_vel = self.forward_vel - control - self.trim_correction
            if abs(dist - self.left_dist) < 0.1 or abs(dist - self.right_dist) < 0.1:
                self.command_motors(0.25*left_vel, 0.25*right_vel)
            else:
                self.command_motors(left_vel, right_vel)
            self.calculate_pose()
            if VERBOSE > 0:
                print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
            
            rate.sleep()
        
        self.command_motors(0.0, 0.0)
        # calculate new pose
        self.calculate_pose()
        confirm_str = f"done:forward:{dist}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done moving forward:", dist)

    def _rotate_robot(self, degrees):
        """
        Rotate the robot by degrees degrees
        """
        self.reset_encoder_values()
        self.calculate_dist_traveled()
        self.pid_controller = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd, 
            sample_time=self.sample_time, 
            output_limits=self.output_limits)
        rate = rospy.Rate(100)
        # Figure out the reverse kinematics 
        # calculating the arc length for each wheel
        target_arc_dist = abs(self.L * degrees * math.pi / 180.0)
        if VERBOSE > 0:
            print("Target arc dist:", target_arc_dist)
        # left turn
        if degrees > 0:
            left_dist_diff = target_arc_dist
            right_dist_diff = target_arc_dist
            while left_dist_diff > 0.01 and right_dist_diff > 0.01:
                left_vel = -self.rotation_vel
                right_vel = self.rotation_vel
                if left_dist_diff < 0.05:
                    right_val = 0.25*right_vel
                if right_dist_diff < 0.05:
                    left_val = 0.25*left_vel
                # speed reduction
                if left_dist_diff <= 0.01:
                    left_vel = 0
                if right_dist_diff <= 0.01:
                    right_vel = 0
                self.command_motors(left_vel, right_vel)
                left_dist_diff = target_arc_dist + self.left_dist
                right_dist_diff = target_arc_dist - self.right_dist
                if VERBOSE > 0:
                    print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
                self.calculate_pose()
                rate.sleep()
        # right turn
        elif degrees < 0:
            left_dist_diff = target_arc_dist
            right_dist_diff = target_arc_dist
            while left_dist_diff > 0.01 and right_dist_diff > 0.01:
                left_vel = self.rotation_vel
                right_vel = -self.rotation_vel
                # speed reduction
                if left_dist_diff < 0.05:
                    right_val = 0.25*right_vel
                if right_dist_diff < 0.05:
                    left_val = 0.25*left_vel
                if left_dist_diff <= 0.01:
                    left_vel = 0
                if right_dist_diff <= 0.01:
                    right_vel = 0
                self.command_motors(left_vel, right_vel)
                left_dist_diff = target_arc_dist - self.left_dist
                right_dist_diff = target_arc_dist + self.right_dist
                if VERBOSE > 0:
                    print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
                self.calculate_pose()
                rate.sleep()

        self.command_motors(0.0,0.0)
        # calculate pose
        self.calculate_pose()
        confirm_str = f"done:rotation:{degrees}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done rotation:", degrees, "degrees")

    def _arc_robot(self, degrees, radius):
        """
        Move the robot in a circular direction
        """
        self.reset_encoder_values()
        self.calculate_dist_traveled()
        self.pid_controller = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd, 
            sample_time=self.sample_time, 
            output_limits=self.output_limits)
        rate = rospy.Rate(100)
        # Figure out the reverse kinematics 
        # calculating the arc length for each wheel
        inner_arc_dist = abs((radius - self.L) * degrees * math.pi / 180.0)
        outer_arc_dist = abs((radius + self.L) * degrees * math.pi / 180.0)
        if VERBOSE > 0:
            print("Target inner arc dist:", inner_arc_dist)
            print("Target outer arc dist:", outer_arc_dist)
        # left turn
        if degrees > 0:
            left_dist_diff = inner_arc_dist + self.left_dist
            right_dist_diff = outer_arc_dist - self.right_dist
            vel_ratio = radius
            while left_dist_diff > 0.001 and right_dist_diff > 0.001:
                left_vel = self.rotation_vel * vel_ratio
                right_vel = self.rotation_vel
                if left_dist_diff <= 0.001:
                    left_vel = 0
                if right_dist_diff <= 0.001:
                    right_vel = 0
                self.command_motors(left_vel, right_vel)
                left_dist_diff = inner_arc_dist + self.left_dist
                right_dist_diff = outer_arc_dist - self.right_dist
                if VERBOSE > 0:
                    print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
                self.calculate_pose()
                rate.sleep()
        # right turn
        elif degrees < 0:
            left_dist_diff = outer_arc_dist - self.left_dist
            right_dist_diff = inner_arc_dist + self.right_dist
            vel_ratio = radius
            while left_dist_diff > 0.001 and right_dist_diff > 0.001:
                left_vel = self.rotation_vel
                right_vel = self.rotation_vel * vel_ratio
                if left_dist_diff < 0.001:
                    left_vel = 0
                if right_dist_diff < 0.001:
                    right_vel = 0
                self.command_motors(left_vel, right_vel)
                left_dist_diff = outer_arc_dist - self.left_dist
                right_dist_diff = inner_arc_dist + self.right_dist
                if VERBOSE > 0:
                    print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
                self.calculate_pose()
                rate.sleep()

        self.command_motors(0.0,0.0)
        confirm_str = f"done:arc:{degrees}:{radius}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done rotation:", degrees, "degrees")

    def command_motors(self, left, right):
        """ This function is called periodically to send motor commands.
        """

        motor_cmd = WheelsCmdStamped()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.vel_left = left
        motor_cmd.vel_right = right   
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
