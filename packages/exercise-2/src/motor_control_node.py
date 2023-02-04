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
import message_filters
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
        ## From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py
        ## Subscribers to the wheel encoders
        self.sub_encoder_ticks_left = message_filters.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped)
        self.sub_encoder_ticks_right = message_filters.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped)
        ## Subscribers to the state control node
        self.sub_state_control_comm = rospy.Subscriber(f'/state_control_node/command', String, self.cb_state_control_comm)

        ## Setup the time syncs 
        self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.5
        )

        self.ts_encoders.registerCallback(self.cb_encoder_data)

        # For PID controller
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

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

        ##
        self.ticks_per_meter = 656

        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None        

        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.tv = 0.0
        self.rv = 0.0
        
        # Setup timer
        self.publish_hz = 10
        self.timer = rospy.Timer(rospy.Duration(1 / self.publish_hz), self.cb_timer)
        self._print_time = 0
        self._print_every_sec = 30
        ##
        
        
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

    def reset_encoder_values(self):
        self.left_wheel_offset = self.left_wheel_ticks
        self.right_wheel_offset = self.right_wheel_ticks

    def cb_encoder_data(self, left_encoder, right_encoder):
        """
        Update encoder distance information from ticks.

        left_encoder:  Left wheel encoder ticks
        right_encoder: Right wheel encoder ticks
        """
        timestamp_now = rospy.get_time()

        # Use the average of the two encoder times as the timestamp
        left_encoder_timestamp = left_encoder.header.stamp.to_sec()
        right_encoder_timestamp = right_encoder.header.stamp.to_sec()
        timestamp = (left_encoder_timestamp + right_encoder_timestamp) / 2

        if not self.left_encoder_last:
            self.left_encoder_last = left_encoder
            self.right_encoder_last = right_encoder
            self.encoders_timestamp_last = timestamp
            self.encoders_timestamp_last_local = timestamp_now
            return

        # Skip this message if the time synchronizer gave us an older message
        dtl = left_encoder.header.stamp - self.left_encoder_last.header.stamp
        dtr = right_encoder.header.stamp - self.right_encoder_last.header.stamp
        if dtl.to_sec() < 0 or dtr.to_sec() < 0:
            self.loginfo("Ignoring stale encoder message")
            return

        left_dticks = left_encoder.data - self.left_encoder_last.data
        right_dticks = right_encoder.data - self.right_encoder_last.data

        left_distance = left_dticks * 1.0 / self.ticks_per_meter
        right_distance = right_dticks * 1.0 / self.ticks_per_meter

        # Displacement in body-relative x-direction
        distance = (left_distance + right_distance) / 2

        # Change in heading
        dyaw = (right_distance - left_distance) / (2 * self.L)

        dt = timestamp - self.encoders_timestamp_last

        if dt < 1e-6:
            self.logwarn("Time since last encoder message (%f) is too small. Ignoring" % dt)
            return

        self.tv = distance / dt
        self.rv = dyaw / dt

        dist = self.tv * dt
        dyaw = self.rv * dt

        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.timestamp = timestamp


        self.left_encoder_last = left_encoder
        self.right_encoder_last = right_encoder
        self.encoders_timestamp_last = timestamp
        self.encoders_timestamp_last_local = timestamp_now

    
    def cb_timer(self, _):
        need_print = time.time() - self._print_time > self._print_every_sec
        if self.encoders_timestamp_last:
            dt = rospy.get_time() - self.encoders_timestamp_last_local
            if abs(dt) > self.encoder_stale_dt:
                if need_print:
                    self.logwarn(
                        "No encoder messages received for %.2f seconds. "
                        "Setting translational and rotational velocities to zero" % dt
                    )
                self.rv = 0.0
                self.tv = 0.0
        else:
            if need_print:
                self.logwarn(
                    "No encoder messages received. " "Setting translational and rotational velocities to zero"
                )
            self.rv = 0.0
            self.tv = 0.0

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
            if VERBOSE > 0:
                print("left wheel:", self.left_dist, "- right wheel:", self.right_dist)
            
            rate.sleep()
        
        self.command_motors(0.0, 0.0)
        # calculate new pose
        
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
                
                rate.sleep()

        self.command_motors(0.0,0.0)
        # calculate pose
        
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

    @staticmethod
    def angle_clamp(theta):
        if theta > 2 * math.pi:
            return theta - 2 * math.pi
        elif theta < -2 * math.pi:
            return theta + 2 * math.pi
        else:
            return theta
if __name__ == '__main__':
    node = MotorControlNode(node_name='motor_control_node')
    # Keep it spinning to keep the node alive
    # main loop
    rospy.spin()
