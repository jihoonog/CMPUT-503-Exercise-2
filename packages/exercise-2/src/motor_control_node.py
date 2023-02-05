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
import typing
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
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self.L = 0.05
        self.update_rate = 30 # 30 Hz

        # Velocity settings
        self.forward_vel = 0.5
        self.rotation_vel = 0.25

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
            self.loginfo("Ignoring stale encoder message")
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


        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.timestamp = timestamp


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
            new_x = self.x_target_hist[-1] + float(dist) * math.cos(self.yaw)
            new_y = self.y_target_hist[-1] + float(dist) * math.sin(self.yaw)
            new_yaw = self.yaw_target_hist[-1]
        elif dist is None:
            new_x = self.x_target_hist[-1]
            new_y = self.y_target_hist[-1]
            rad = float(dist) * math.pi / 180.0
            new_yaw = self.angle_clamp(self.yaw + rad)
        else:
            rad = float(dist) * math.pi / 180.0
            new_yaw = self.angle_clamp(self.yaw + rad)
            new_x = self.x_target_hist[-1] + float(dist) * math.cos(new_yaw)
            new_y = self.y_target_hist[-1] + float(dist) * math.sin(new_yaw)

        self.x_target_hist.append(new_x)
        self.y_target_hist.append(new_y)
        self.yaw_target_hist.append(new_yaw)
        self.time_target_hist.append(rospy.get_time())

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
        target_vec = (end_x_w - start_x_w, end_y_w - start_y_w)

        while self._get_dist_to_target((x_r, y_r), (end_x_w, end_y_w)) > 0.05:
            robot_vec = (x_r - start_x_w, y_r - start_y_w)
            track_error_vec = self._get_orthogonal_projection(robot_vec, target_vec)
            error_dist = self._get_mag(track_error_vec)
            error_direction = self._2d_cross_product(target_vec, (x_r, y_r))

            # right 
            if error_direction > 0:
                error_dist = error_dist
            # left
            elif error_direction < 0:
                error_dist = -error_dist
            else:
                error_dist = 0

            if VERBOSE > 0:
                print('dist_drift:', error_dist)

            correction = self.pid_controller(error_dist)

            left_vel = self.forward_vel + correction 
            right_vel = self.forward_vel - correction
            # Slow down to mitigate overshooting
            if self._get_dist_to_target((x_r, y_r), (end_x_w, end_y_w)) < 0.15:
                self.command_motors(0.25*left_vel, 0.25*right_vel)
            else:
                self.command_motors(left_vel, right_vel)

            if VERBOSE > 0:
                print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)
            rate.sleep()
            x_r = self.x
            y_r = self.y
        
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
        # self.pid_controller = PID(
        #     Kp=self.Kp,
        #     Ki=self.Ki,
        #     Kd=self.Kd, 
        #     sample_time=self.sample_time, 
        #     output_limits=self.output_limits)

        rate = rospy.Rate(self.update_rate)
        target_yaw_w = self.yaw_target_hist[-1]
        target_rad_w = target_yaw_w * math.pi / 180.0
        if VERBOSE > 0:
            print("Target yaw :", target_yaw_w)
        # left turn
        if degrees > 0:
            while math.fabs(target_yaw_w - (self.yaw * 180.0 / math.pi)) > 10.0:
                left_vel = -self.rotation_vel
                right_vel = self.rotation_vel
                if math.fabs(target_yaw_w - (self.yaw * 180.0 / math.pi)) > 25.0:
                    left_vel = 0.25*left_vel 
                    right_vel = 0.25*right_vel
                self.command_motors(left_vel, right_vel)
                if VERBOSE > 0:
                    print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)

                
                rate.sleep()
        # right turn
        elif degrees < 0:
            while math.fabs(target_yaw_w - (self.yaw * 180.0 / math.pi)) > 10.0:
                left_vel = self.rotation_vel
                right_vel = -self.rotation_vel
                if math.fabs(target_yaw_w - (self.yaw * 180.0 / math.pi)) > 25.0:
                    left_vel = 0.25*left_vel 
                    right_vel = 0.25*right_vel
                self.command_motors(left_vel, right_vel)
                if VERBOSE > 0:
                    print("x:", self.x, "- y:", self.y, '- theta:', self.yaw * 180.0 / math.pi)
                    
                rate.sleep()

        self.command_motors(0.0,0.0)
        
        confirm_str = f"done:rotation:{degrees}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done rotation:", degrees, "degrees")

    def _arc_robot(self, degrees, radius):
        """
        Move the robot in a circular direction
        """
        pass
        # self.pid_controller = PID(
        #     Kp=self.Kp,
        #     Ki=self.Ki,
        #     Kd=self.Kd, 
        #     sample_time=self.sample_time, 
        #     output_limits=self.output_limits)
        rate = rospy.Rate(self.update_rate)
        # Figure out the reverse kinematics 
        # calculating the arc length for each wheel

        self.command_motors(0.0,0.0)
        confirm_str = f"done:arc:{degrees}:{radius}"
        self.pub_executed_commands.publish(confirm_str)
        if VERBOSE > 0:
            print("Done rotation:", degrees, "degrees")

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
