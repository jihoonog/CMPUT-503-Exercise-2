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
import time
import message_filters
import typing
from PID import PID


from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, String, Float64MultiArray,Float32MultiArray

import rosbag


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
        self.veh_name = "csc22935"

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self.L = 0.05
        self.update_rate = 30 # 30 Hz

        # Velocity settings
        self.forward_vel = 0.6
        self.rotation_vel = 0.6

        # Subscribers
        ## Subscribers to the state control node
        self.sub_state_control_comm = rospy.Subscriber(f'/state_control_node/command', String, self.cb_state_control_comm)

        ## From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py
        ## Subscribers to the wheel encoders
        self.sub_encoder_ticks_left = message_filters.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped)
        self.sub_encoder_ticks_right = message_filters.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped)
        ## Setup the time syncs 
        self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.5
        )

        self.ts_encoders.registerCallback(self.cb_encoder_data)

        # For PID controller
        self.Kp = 0.4
        self.Ki = 0.075
        self.Kd = 0.00
        self.sample_time = 1.0/self.update_rate
        self.output_limits = (-0.3, 0.3)
        self.pid_controller = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd, 
            sample_time=self.sample_time, 
            output_limits=self.output_limits)

        # Store the target state for each command received
        # Store the initial state for startup
        self.x_target_hist = [0]
        self.y_target_hist = [0]
        self.yaw_target_hist = [math.pi/2.0]
        self.time_target_hist = [rospy.get_time()]

        ##
        self.ticks_per_meter = 656
        self.encoder_stale_dt = 1.0 

        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None        

        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = math.pi/2.0
        self.tv = 0.0
        self.rv = 0.0
        self.lwv = 0.0
        self.rwv = 0.0
        
        # Setup timer
        self.publish_hz = self.update_rate
        self.timer = rospy.Timer(rospy.Duration(1 / self.publish_hz), self.cb_timer)
        self._print_time = 0
        self._print_every_sec = 30
        ##
        
        
        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        ## Publish what wheel commands have been executed
        self.pub_executed_commands = rospy.Publisher(f'/wheels_driver_node/wheels_cmd_executed', String, queue_size = 1)

        self.pub_world_frame = rospy.Publisher(f'/motor_control_node/world_frame_coord', Pose2DStamped, queue_size = 1)
        
        self.log("Initialized")

    # Start of callback functions
    def cb_encoder_data(self, left_encoder, right_encoder):
        """
        Update encoder distance information from ticks.
        From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py#L101

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
            # self.loginfo("Ignoring stale encoder message")
            return

        left_dticks = left_encoder.data - self.left_encoder_last.data
        right_dticks = right_encoder.data - self.right_encoder_last.data

        left_distance = left_dticks * 1.0 / self.ticks_per_meter
        right_distance = right_dticks * 1.0 / self.ticks_per_meter

        # Displacement in body-relative x-direction
        dist = (left_distance + right_distance) / 2

        # Change in heading
        dyaw = (right_distance - left_distance) / (2 * self.L)

        dt = timestamp - self.encoders_timestamp_last

        if dt < 1e-6:
            self.logwarn("Time since last encoder message (%f) is too small. Ignoring" % dt)
            return

        self.tv = dist / dt
        self.rv = dyaw / dt
        self.lwv = left_distance / dt
        self.rwv = right_distance / dt


        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.timestamp = timestamp

        f = Pose2DStamped()

        f.x = self.x
        f.y = self.y
        f.theta = self.yaw

        f.header.stamp = rospy.Time.now()
        self.pub_world_frame.publish(f)

        self.left_encoder_last = left_encoder
        self.right_encoder_last = right_encoder
        self.encoders_timestamp_last = timestamp
        self.encoders_timestamp_last_local = timestamp_now

    
    def cb_timer(self, _):
        """
        Callback for the timer

        From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py#L173
        """
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
            self.get_new_target_world_frame(rotate=None, dist=dist)
            self._move_forward(dist)
        elif command_args[0].lower() == "right":
            rotation = float(command_args[1])
            self.get_new_target_world_frame(rotate=-rotation, dist=None)
            self._rotate_robot(-rotation)
        elif command_args[0].lower() == "left":
            rotation = float(command_args[1])
            self.get_new_target_world_frame(rotate=rotation, dist=None)
            self._rotate_robot(rotation)
        # fix this later
        elif command_args[0].lower() == "arc_left":
            rotation = float(command_args[1])
            radius = float(command_args[2])
            self._arc_robot(rotation, radius)
        elif command_args[0].lower() == "arc_right":
            rotation = float(command_args[1])
            radius = float(command_args[2])
            self._arc_robot(-rotation, radius)
        elif command_args[0].lower() == "shutdown":
            print("Motor Control Node shutting down")
            rospy.signal_shutdown("Motor Control Node Shutdown command received")
        else:
            print("Not a valid command")
    # End of Callback functions

    def get_new_target_world_frame(self, rotate = None, dist = None):
        """
        This returns the new target world frame based on the current target world frame
        and the dist and/or rotation from the current target

        If both rotate and dist is defined then the rotation operation is DONE FIRST before dist

        rotate - rotation in degrees relative to the current target
        dist - distance in meter relative to the current target
        """
        if dist is None and rotate is None:
            self.logwarn("Both dist and rotate is None for get_new_target_world_frame ignoring")
            return None
        
        if rotate is None:
            old_yaw = self.yaw_target_hist[-1]
            new_x = self.x_target_hist[-1] + float(dist) * math.cos(old_yaw)
            new_y = self.y_target_hist[-1] + float(dist) * math.sin(old_yaw)
            new_yaw = old_yaw
        elif dist is None:
            old_yaw = self.yaw_target_hist[-1]
            new_x = self.x_target_hist[-1]
            new_y = self.y_target_hist[-1]
            rad = float(rotate) * math.pi / 180.0
            new_yaw = self.angle_clamp(old_yaw + rad)
        else:
            old_yaw = self.yaw_target_hist[-1]
            rad = float(rotate) * math.pi / 180.0
            new_yaw = self.angle_clamp(old_yaw + rad)
            new_x = self.x_target_hist[-1] + float(dist) * math.cos(new_yaw)
            new_y = self.y_target_hist[-1] + float(dist) * math.sin(new_yaw)

        self.x_target_hist.append(new_x)
        self.y_target_hist.append(new_y)
        self.yaw_target_hist.append(new_yaw)
        self.time_target_hist.append(rospy.get_time())
        print("New target:", new_x, new_y, new_yaw)

    def _move_forward(self, dist):
        """
        Move the robot forward by dist amount
        """
        rate = rospy.Rate(self.update_rate)
        start_x_w = self.x_target_hist[-2]
        start_y_w = self.y_target_hist[-2]
        end_x_w = self.x_target_hist[-1]
        end_y_w = self.y_target_hist[-1]
        x_r = self.x
        y_r = self.y

        self.pid_controller = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd, 
            sample_time=self.sample_time, 
            output_limits=self.output_limits)
        
        dist_to_target = self._get_dist_to_target((self.x, self.y), (end_x_w, end_y_w))
        prev_dist_to_target = dist_to_target
        while dist_to_target > 0.05:
            # The vector to the target point from the robot
            target_vec = (end_x_w - self.x, end_y_w - self.y)
            # The vector the robot's movement
            robot_vec = (math.cos(self.yaw), math.sin(self.yaw))

            dot_prod = np.dot(target_vec, robot_vec)
            target_vec_mag = self._get_mag(target_vec)
            robot_vec_mag = self._get_mag(robot_vec)
            error_angle = math.acos(dot_prod / (robot_vec_mag * target_vec_mag))    
            error_direction = np.cross(target_vec, robot_vec)


            print("x_r:", x_r, "y_r:", y_r)

            # left of target
            if error_direction > 0:
                error_rad = error_angle
            # right of target
            elif error_direction < 0:
                error_rad = -error_angle
            else:
                error_rad = 0

            print('angle error:', error_rad * 180.0 / math.pi)
            
            if VERBOSE > 0:
                pass
            
            correction = self.pid_controller(error_rad)

            left_vel = self.forward_vel - correction 
            right_vel = self.forward_vel + correction
            # Slow down to mitigate overshooting
            if dist_to_target < 0.15:
                self.command_motors(0.5*left_vel, 0.5*right_vel)
                if dist_to_target > prev_dist_to_target:
                    break
            else:
                self.command_motors(left_vel, right_vel)
            
            if VERBOSE > 0:
                print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)
            
            # Have the motor move for a bit
            rate.sleep()
            # Update position and distances to target
            x_r = self.x
            y_r = self.y
            prev_dist_to_target = dist_to_target
            dist_to_target = self._get_dist_to_target((x_r, y_r), (end_x_w, end_y_w))

            
        
        self.command_motors(0.0, 0.0)
        print("Current position:", self.x, self.y, self.yaw * 180.0 / math.pi)
        confirm_str = f"done:forward:{dist}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done moving forward:", dist)

    def _rotate_robot(self, degrees):
        """
        Rotate the robot by degrees degrees
        """
        # self.pid_controller = PID(
        #     Kp=self.Kp,
        #     Ki=self.Ki,
        #     Kd=self.Kd, 
        #     sample_time=self.sample_time, 
        #     output_limits=self.output_limits)

        rate = rospy.Rate(self.update_rate)
        target_rad_w = self.yaw_target_hist[-1]
        if VERBOSE > 0:
            print("Target yaw :", target_rad_w * 180.0 / math.pi)
        # left turn
        if degrees > 0:
            while math.fabs(target_rad_w - self.yaw) > 0.1:
                left_vel = -self.rotation_vel
                right_vel = self.rotation_vel
                self.command_motors(left_vel, right_vel)
                if VERBOSE > 0:
                    print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)

                
                rate.sleep()
        # right turn
        elif degrees < 0:
            while math.fabs(target_rad_w - self.yaw) > 0.1:
                left_vel = self.rotation_vel
                right_vel = -self.rotation_vel
                self.command_motors(left_vel, right_vel)
                if VERBOSE > 0:
                    print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)
                    
                rate.sleep()

        self.command_motors(0.0,0.0)
        
        confirm_str = f"done:rotation:{degrees}"
        print("Current position:", self.x, self.y, self.yaw * 180.0 / math.pi)
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done rotation:", degrees, "degrees")

    def _arc_robot(self, count_limit, radius):
        """
        Move the robot in a circular direction
        """
        rate = rospy.Rate(self.update_rate)
        initial_x = self.x
        initial_x = self.y
        count = 0
        if count_limit > 0:
            while count < abs(count_limit):
                left_vel = self.forward_vel * radius
                right_vel = self.forward_vel
                self.command_motors(left_vel, right_vel)
                if VERBOSE > 0:
                    print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)
                rate.sleep()
                count += 1

        else:
            while count < abs(count_limit):
                left_vel = self.forward_vel
                right_vel = self.forward_vel * radius
                self.command_motors(left_vel, right_vel)
                if VERBOSE > 0:
                    print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)
                rate.sleep()
                count += 1

        self.command_motors(0.0,0.0)
        confirm_str = f"done:arc:{count_limit}:{radius}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done rotation:", count_limit, "count limit")

    def command_motors(self, left, right):
        """ 
        This function is called periodically to send motor commands.
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
    @staticmethod
    def _get_orthogonal_projection(vec_a: tuple, vec_b: tuple) -> tuple:
        """
        This will return the orthogonal projection of vec_a onto vec_b
        i.e., the return vector is orthogonal to vec_b

        vec_a - vector a

        vec_b - vector b
        """
        scalar = sum([i*j for i, j in zip(vec_a, vec_b)]) / sum([i*j for i, j in zip(vec_b, vec_b)])
        proj = [scalar * i for i in vec_b]
        orth = (i - j for i, j in zip(vec_a, proj))
        return orth

    @staticmethod
    def _get_dist_to_target(curr: tuple, target: tuple):
        dist = sum([(j - i)*(j - i) for i, j in zip(curr, target)])
        return math.sqrt(dist)
    
    @staticmethod
    def _get_mag(vec: tuple):
        return math.sqrt(sum([i * i for i in vec]))

    @staticmethod
    def _2d_cross_product(vec_a: tuple, point: tuple):
        return vec_a[0] * point[1] - vec_a[1] * point[0]

if __name__ == '__main__':
    node = MotorControlNode(node_name='motor_control_node')
    # Keep it spinning to keep the node alive
    # main loop
    rospy.spin()
