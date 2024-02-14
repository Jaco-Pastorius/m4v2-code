"""
Python implementation of Altitude Controller

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
from px4_msgs.msg import StateEstimate
from px4_msgs.msg import VehicleLocalPosition

from geometry_msgs.msg import PoseStamped

import numpy as np
from copy import deepcopy

class AltitudeController(Node):
    def __init__(self):
        super().__init__('altitude_controller')

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

        self.vehicle_local_position_subscriber_ = self.create_subscription(
                                            VehicleLocalPosition,
                                            '/fmu/out/vehicle_local_position',
                                            self.vehicle_local_position_callback,
                                            qos_profile_sensor_data)
        self.vehicle_local_position_subscriber_  # prevent unused variable warning

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
        self.state_estimate_publisher_   = self.create_publisher(
                                                    StateEstimate, 
                                                    "/state_estimate", 
                                                    10)

        # Create timer
        self.offboard_setpoint_counter_ = 0
        self.Ts = 0.1  # 100 Hz
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set RC input limits
        self.min  = 1094
        self.dead = 1514
        self.max  = 1934

        # Desired tilt angle
        self.tilt_angle_desired  = 0.0

        # Throttle, roll, pitch, and yaw
        self.throttle      = 0.0

        # Current altitude, descent speed, desired descent speed, and tilt_angle
        self.z = 0.0
        self.zdot = 0.0
        self.zdot_d = 0.0
        self.tilt_angle = 0.0

        # controller gains
        self.ei = 0.0
        self.ki = 0.0
        self.k = 10.0

        # Altitude control logic
        self.altitude = False

        # Controller parameters
        self.m = 5.0   # kg
        self.g = 9.81  # m/s/s
        self.thrust_to_weight_ratio = 2.0 

        # arming flag 
        self.offboard_flag = False
        self.offboard    = False 
        self.offboard_setpoint_counter_ = 0

        # Extended Kalman Filter
        self.x = np.array([0.0,0.0])
        self.u = 0.0                    # current thrust
        self.P = np.diag([0.1,0.1])

    # usage: start in manual/stabilize mode and fly up. At that point flip the switch which will take you into offboard mode
    def timer_callback(self):

        # run ekf 
        # self.ekf(self.z,self.u)
        # print(f"zdot:{self.zdot}")
        # print(f"zdot_d:{self.zdot_d}")

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
            pass 
            if self.altitude:
                pass
                # Update controller
                self.controller_update()

                # Publish the desired tilt angle 
                # self.publish_tilt_angle_ref()

            # print("publishing vehicle attitude setpoint")
            # self.publish_vehicle_attitude_setpoint()

    def vehicle_attitude_callback(self, msg):
        self.roll,self.pitch,self.yaw = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])

    def tilt_angle_callback(self, msg):
        self.tilt_angle = msg.value

    def rc_listener_callback(self, msg):

        self.u = self.throttle_to_thrust(self.normalize(msg.values[2]))

        # set current desired descent speed (set this to what normally controls pitch)
        self.zdot_d = 2*(self.normalize(msg.values[1])-0.5)  # between -1 and 1 (pitch is inverted hence the minus sign) (-1 means going up !!!)
        # print(f"self.zdot_d: {self.zdot_d}")

        # get arm command
        if (msg.values[8] == self.max):
            self.offboard_flag = True
        else:
            self.offboard_flag = False

        if (msg.values[7] == self.max):
            self.altitude = True
        else:
            self.altitude = False

    def vehicle_local_position_callback(self, msg):
        self.zdot = msg.vz
        
    def controller_update(self):
        e = - (self.zdot - self.zdot_d)
        self.ei += e*self.Ts
        self.u = - self.k * e - self.ki * self.ei
        self.throttle = self.thrust_to_throttle(self.u)

        print(f"e:, throttle {e}, {self.throttle}")

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

    def f(self,x,u):
        return np.array([x[0] + self.Ts*x[1],x[1] + self.Ts*(u/self.m - self.g)])
    
    def h(self,x):
        return x[0]

    def ekf(self,z,u):
        Q = 0.01 * np.diag([1,1])
        R = 1e-5
        F = np.array([[1,self.Ts],[0,1]])
        H = np.array([1,0])

        print(f"u: {u}")
        print(f"z: {z}")

        # Predict
        xkkm1 = self.f(self.x,u)
        Pkkm1 = F @ self.P @ F.transpose() + Q

        print(f"xkkm1: {xkkm1}")
        print(f"Pkkm1: {Pkkm1}")

        # Update 
        y = z - self.h(xkkm1)
        S = H @ Pkkm1 @ H.transpose() + R
        invS = 1/S
        K = Pkkm1 @ H.transpose() * invS
        print(f"K: {K}")
        self.x = xkkm1 + K*y
        self.P = (np.eye(self.x.shape[0]) - K@H)@Pkkm1

        # Publish state estimate
        print(f"zhat: {self.x[0]}")
        print(f"zdot_hat: {self.x[1]}")
        msg = StateEstimate()
        msg.state[0] = self.x[0]
        msg.state[1] = self.x[1]
        # msg.state[2] = self.x[2]
        self.state_estimate_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print("Starting altitude controller...\n")
    offboard_control = AltitudeController()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
