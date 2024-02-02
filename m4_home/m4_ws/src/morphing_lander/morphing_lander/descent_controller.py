"""
Python implementation of Descent Controller

"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import TiltAngle
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import InputRc

from geometry_msgs.msg import PoseStamped

import numpy as np
from copy import deepcopy

class DescentController(Node):
    def __init__(self):
        super().__init__('descent_controller')

        # Subscriptions
        self.rc_input_subscriber_ = self.create_subscription(
                                            InputRc,
                                            '/fmu/out/input_rc',
                                            self.rc_listener_callback,
                                            qos_profile_sensor_data)
        
        # To Do: add mocap callback 
        # self.mocap_subscriber = self.create_subscription(
        #                                     PoseStamped,
        #                                     '/vrpn_mocap/m4_base/pose',
        #                                     self.mocap_pose_callback,
        #                                     10)


        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
                                                        OffboardControlMode, 
                                                        "/fmu/in/offboard_control_mode", 
                                                        10)
        self.vehicle_command_publisher_       = self.create_publisher(
                                                        VehicleCommand, 
                                                        "/fmu/in/vehicle_command", 
                                                        10)
        self.vehicle_attitude_setpoint_publisher_   = self.create_publisher(
                                                        VehicleAttitudeSetpoint, 
                                                        "/fmu/in/vehicle_attitude_setpoint", 
                                                        10)
        self.tilt_angle_ref_external_publisher   = self.create_publisher(
                                                        TiltAngle, 
                                                        "/tilt_angle_ref_external", 
                                                        10)
        
        # Create timer
        self.offboard_setpoint_counter_ = 0
        self.Ts = 0.01  # 100 Hz
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set RC input limits
        self.min  = 1094
        self.dead = 1514
        self.max  = 1934

        # Desired thrust and tilt angle 
        self.throttle_desired       = 0.0
        self.throttle_filtered      = 0.0
        self.tilt_angle_desired     = 0.0

        # previous desired throttle for low-pass filtering
        self.throttle_desired_prev  = 0.0
        self.alpha = 0.5                      # low pass filter gain

        # Current descent speed 
        self.zdot = 0.0

        # Descent logic
        self.land_begin = False
        self.t0_set = False
        self.land_interrupted = False

        # Controller parameters
        self.m = 5.0   # kg
        self.g = 9.81  # m/s/s
        self.thrust_to_weight_ratio = 1.5 # be conservative to start out
        self.Vsq = 0.5 # m/s : start with a low descent rate initially 

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

            # Arm the vehicle (vehicle should already be armed here)
            self.arm()  # DISABLE THIS WHEN DOING REAL TESTS: will already be in stabilize mode

        # Give offboard control mode the necessary heartbeat
        self.publish_offboard_control_mode()

        if self.land_begin:

            # Compute desired tilt_angle 
            self.compute_desired_tilt_angle()

            # Publish the desired tilt angle 
            self.publish_tilt_angle_ref()

            # Publish the desired throttle and roll, pitch, yaw (set these to zero)
            self.publish_vehicle_attitude_setpoint()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def rc_listener_callback(self, msg):

        # get desired throttle and use a low pass filter to smooth out the inputs
        self.throttle_desired_prev = deepcopy(self.throttle_desired)
        self.throttle_desired = self.normalize(msg.values[2])
        self.throttle_desired = self.alpha * self.throttle_desired + (1.0-self.alpha) * self.throttle_desired_prev

        if (msg.values[8] == self.max):
            self.land_begin = True
        else:
            self.land_begin = False

    # Compute desired tilt angle
    def compute_desired_tilt_angle(self):

        # set current descent velocity
        zdotk = -self.Vsq # for test without optitrack (To Do: replace this with actual mocap values)

        # convert desired throttle to desired thrust 
        thrust_desired = self.throttle_to_thrust(self.throttle_desired)

        # compute minimum allowable thrust
        T_min = self.m*self.g - self.m*(zdotk+self.Vsq)/self.Ts

        # filter desired thrust and throttle
        thrust_filtered = deepcopy(thrust_desired)
        self.throttle_filtered = deepcopy(self.throttle_desired)

        if (thrust_desired < T_min) : 
            self.throttle_filtered = self.thrust_to_throttle(T_min)
            thrust_filtered = T_min

        # compute the desired tilt angle 
        self.tilt_angle_desired = np.arccos((self.m*self.g-self.m*(zdotk + self.Vsq)/self.Ts)/thrust_filtered)

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    # Give offboard control a heartbeat
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False  # True for position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_vehicle_attitude_setpoint(self):
        msg = VehicleAttitudeSetpoint()
        msg.roll_body = 0.0
        msg.pitch_body = 0.0
        msg.yaw_body = 0.0 
        msg.thrust_body = [0.0, 0.0, -self.throttle_filtered]  # For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_attitude_setpoint_publisher_.publish(msg)       
        
    def publish_tilt_angle_ref(self):
        if not self.land_interrupted:
            msg = TiltAngle()
            msg.value = 0.0

            if self.land_begin:
                # get current time in seconds
                t = Clock().now().nanoseconds / 1e9
                msg.value = float(self.tilt_angle_desired)

            self.tilt_angle_ref_external_publisher.publish(msg)
        else:
            pass
    
    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def normalize(self,rc_in):
        return (rc_in-self.min)/(self.max-self.min)

    def throttle_to_thrust(self,throttle):
        return self.thrust_to_weight_ratio * self.m * self.g * throttle
        
    def thrust_to_throttle(self,thrust):
        return thrust / self.thrust_to_weight_ratio * self.m * self.g


def main(args=None):
    rclpy.init(args=args)
    print("Starting descent controller...\n")
    offboard_control = DescentController()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
