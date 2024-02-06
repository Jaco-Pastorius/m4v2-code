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
from px4_msgs.msg import VehicleAttitude

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
        self.rc_input_subscriber_  # prevent unused variable warning

        self.vehicle_attitude_subscriber_ = self.create_subscription(
                                            VehicleAttitude,
                                            '/fmu/out/vehicle_attitude',
                                            self.vehicle_attitude_callback,
                                            qos_profile_sensor_data)
        self.vehicle_attitude_subscriber_  # prevent unused variable warning


        self.tilt_angle_subscriber_ = self.create_subscription(
                                            TiltAngle,
                                            '/fmu/in/tilt_angle',
                                            self.tilt_angle_callback,
                                            qos_profile_sensor_data)
        self.tilt_angle_subscriber_  # prevent unused variable warning

        self.mocap_subscriber = self.create_subscription(
                                            PoseStamped,
                                            '/vrpn_mocap/m4_base/pose',
                                            self.mocap_pose_callback,
                                            10)

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
        self.Ts = 0.01  # 10 Hz
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set RC input limits
        self.min  = 1094
        self.dead = 1514
        self.max  = 1934

        # Desired tilt angle
        self.tilt_angle_desired     = 0.0

        # Throttle, roll, pitch, and yaw
        self.throttle      = 0.0
        self.roll          = 0.0
        self.pitch         = 0.0
        self.yaw           = 0.0

        # Current descent speed and desired descent speed
        self.zdot = 0.0
        self.zdot_d = 0.0

        # Current tilt angle
        self.tilt_angle = 0.0

        # Descent logic
        self.descent = False

        # Controller parameters
        self.m = 5.0   # kg
        self.g = 9.81  # m/s/s
        self.thrust_to_weight_ratio = 2.0 
        self.k = 0.01 # controller gain

        # arming flag 
        self.offboard_flag = False
        self.offboard    = False 
        self.offboard_setpoint_counter_ = 0


    # usage: start in manual/stabilize mode and fly up. At that point flip the switch which will take you into offboard mode
    def timer_callback(self):

        # Go into offboard mode
        if (self.offboard_flag):
            # publish offboard mode heartbeat
            self.publish_offboard_control_mode()  

            if not self.offboard:
                if (self.offboard_setpoint_counter_ == 10):
                    # go to offboard mode after 1 second
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

                    # Arm the vehicle
                    self.arm()

                    # vehicle is now in offboard mode and armed
                    self.offboard = True

                # stop the counter after reaching 11
                if (self.offboard_setpoint_counter_ < 11):
                    self.offboard_setpoint_counter_ += 1

        else:
            if self.offboard:
                self.disarm()  
                self.offboard = False 
                self.offboard_setpoint_counter_ = 0
        
        if self.offboard:

            if self.descent:
                # Update controller
                self.controller_update()

                # Publish the desired tilt angle 
                self.publish_tilt_angle_ref()

            print("publishing vehicle attitude setpoint")
            self.publish_vehicle_attitude_setpoint()

    def vehicle_attitude_callback(self, msg):
        self.roll,self.pitch,self.yaw = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])

    def tilt_angle_callback(self, msg):
        self.tilt_angle = msg.value

    def rc_listener_callback(self, msg):

        # set current desired descent speed (set this to right LR lever !!!!!!!!)
        self.zdot_d = msg.value[]

        # get arm command
        if (msg.values[8] == self.max):
            self.offboard_flag = True
        else:
            self.offboard_flag = False

        # get current desired throttle
        self.throttle = self.normalize(msg.values[2])
        
        # get current desired roll, pitch, yaw
        self.roll     = self.normalize(msg.values[0])
        self.pitch    = self.normalize(msg.values[1])
        self.yaw      = self.normalize(msg.values[3])

        if (msg.values[7] == self.max):
            self.descent = True
        else:
            self.descent = False

    def mocap_pose_callback(self, msg):
        
    # Compute desired tilt angle
    def controller_update(self):

        thrust = self.m/np.cos(np.deg2rad(self.tilt_angle)) * (self.g - self.k * (self.zdot - self.zdot_d))
        self.throttle_filtered = self.thrust_to_throttle(thrust)

        # compute the desired tilt angle 
        self.tilt_angle_desired = np.arccos(1/self.thrust_to_weight_ratio*(1-self.k/self.g*(self.zdot - self.zdot_d)))

        # finally set roll pitch yaw filtered to zero
        self.roll_filtered  = 0.0
        self.pitch_filtered = 0.0
        self.yaw_filtered   = 0.0

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
        msg.q_d[0] = 1.0
        msg.q_d[1] = 0.0
        msg.q_d[2] = 0.0
        msg.q_d[3] = 0.0

        msg.thrust_body =  [0.0,0.0,-self.throttle]
        print(f"msg.thrust_body{msg.thrust_body}")
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_attitude_setpoint_publisher_.publish(msg)   

    def publish_tilt_angle_ref(self):
        msg = TiltAngle()
        msg.value = np.rad2deg(self.tilt_angle_desired)
        self.tilt_angle_ref_external_publisher.publish(msg)
    
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

    # VehicleAttitudeSetpoint conversions
    def normalize(self,rc_in):
        # returns values between -1 and 1 
        return (rc_in-self.min)/(self.max-self.min)

    def throttle_to_thrust(self,throttle):
        return self.thrust_to_weight_ratio * self.m * self.g * throttle
        
    def thrust_to_throttle(self,thrust):
        return  np.clip(thrust / (self.thrust_to_weight_ratio * self.m * self.g),0.0,1.0)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw



def main(args=None):
    rclpy.init(args=args)
    print("Starting descent controller...\n")
    offboard_control = DescentController()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
