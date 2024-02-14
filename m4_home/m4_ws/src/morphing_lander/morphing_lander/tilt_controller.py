"""
Python implementation of Tilt Controller

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc, TiltAngle

import numpy as np
from copy import deepcopy

# roboclaw
from morphing_lander.roboclaw_3 import Roboclaw

class TiltController(Node):
    def __init__(self):
        super().__init__('tilt_controller')

        # RC subscription
        self.rc_subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.rc_subscription  # prevent unused variable warning

        # External tilt angle ref subscription
        self.tilt_angle_ref_external_subscription = self.create_subscription(
            TiltAngle,
            '/tilt_angle_ref_external',
            self.tilt_angle_ref_external_listener_callback,
            qos_profile_sensor_data)
        self.tilt_angle_ref_external_subscription  # prevent unused variable warning

        # Publisher tilt angle
        self.publisher_tilt = self.create_publisher(TiltAngle, '/fmu/in/tilt_angle', 10)

        # Timer
        self.Ts = 0.1  # 20 Hz
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set min, max, and dead values for LS_in (the control for the tilt angle coming from rc input)
        self.min = 1094 # corresponds to zero degrees i.e. fly configuration
        self.max = 1934 # corresponds to 90 degrees i.e. drive configuration
        self.dead = 1514 # corresponds to midway through i.e. 45 degrees

        # Initialize LS_in
        self.LS_in = 1514

        # Manual vs automatic control (start as manual because need to initalize encoder by bringing manually to drive and flipping the encoder switch)
        self.manual = True

        # Initialize current tilt_angle and tilt_angle_ref_external to zero
        self.tilt_angle = 0.0
        self.tilt_angle_prev = 0.0
        self.tilt_angle_vel  = 0.0
        self.tilt_angle_ref_external = 0.0

        # Initialize roboclaw at given address
        self.address = 0x80
        self.rc = Roboclaw("/dev/ttyACM1",115200)
        self.rc.Open()

        # Set encoder count to zero (always start robot in drive configuration)
        self.rc.SetEncM2(self.address,0)
        self.reset_encoder = 0

        # Set pin functions for motor 2 (M2) to go to zero when it reaches home (limit switch)
        self.rc.SetPinFunctions(self.address,0x00,0x62,0x62)

        # Encoder data for tilt angle publishing (manually converting between data which was collected for encoder 44 from pololu to encoder 45. I should recalibrate)
        counts_per_rev_45 = 2248.86
        counts_per_rev_44 = 1632.67    
        self.encoder_data = np.array([0,2518,4902,6813,8429,9396,10477,11156,13324,14293,15396,16820,17563,19043]) * counts_per_rev_45/counts_per_rev_44
        self.angle_data = np.array([88,85,78.4,70.7,62.7,57.3,51.2,47.1,33.8,27.9,21.3,12.7,8.3,0.5])

        # Declare ros2 parameters: control gains
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.035),
                ('kd', 0.0), 
                ('ki', 0.0)
            ]
        )

        # Retrieve parameter values
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value

        # Initialize integral error
        self.ei = 0.0

        # limit tilt_angle 
        self.limit_tilt = 0.5 # degrees

    def rc_listener_callback(self, msg):
        # https://futabausa.com/wp-content/uploads/2018/09/18SZ.pdf
        self.LS_in = msg.values[9] # corresponds to LS trim selector on futaba T18SZ that I configured in the function menu

        # reset encoder 
        self.reset_encoder = msg.values[12]

        # set manual or automatic control of tilt angle
        if msg.values[7] == self.max:
            self.manual = False
        else:
            self.manual = True

    def tilt_angle_ref_external_listener_callback(self, msg):
        # set previous tilt angle
        self.tilt_angle_prev = deepcopy(self.tilt_angle)
        # set current tilt angle
        self.tilt_angle_ref_external = msg.value # in degrees

    def timer_callback(self):
        if (self.reset_encoder == self.max):
            self.rc.SetEncM2(self.address,0)

        # compute and print current tilt angle
        enc_count = self.rc.ReadEncM2(self.address)
        self.tilt_angle =  float(np.interp(enc_count[1],self.encoder_data,self.angle_data))
        print(f"tilt angle is: {self.tilt_angle}")

        # publish tilt angle
        msg = TiltAngle()
        msg.value = self.tilt_angle
        self.publisher_tilt.publish(msg)

        # compute tilt angle velocity
        self.tilt_angle_vel = (self.tilt_angle - self.tilt_angle_prev)/self.Ts

        if (self.manual):
            # manual control of tilt angle
            u = self.normalize(self.LS_in)
            self.spin_motor(u)
        else:
            ref = self.tilt_angle_ref_external
            u = self.control(ref)
  
    def normalize(self,LS_in):
        return (LS_in-self.dead)/(self.max-self.dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def stop(self):
        self.rc.ForwardM2(self.address,self.map_speed(0.0))

    def spin_motor(self, tilt_speed):
        # takes in a tilt_speed between -1 and 1 writes the pwm signal to the roboclaw 

        # limit speed when reaching the limit tilt angle
        if (self.tilt_angle < self.limit_tilt):
            if (tilt_speed < 0) : tilt_speed = 0.0
            else: pass  

        motor_speed = self.map_speed(abs(tilt_speed))
        if (tilt_speed < 0): # go up
            if motor_speed >= 127: 
                motor_speed = 126 # weird bug not sure why this is needed
            self.rc.ForwardM2(self.address, motor_speed)
        else:
            if motor_speed >= 127:
                motor_speed = 126
            self.rc.BackwardM2(self.address, motor_speed)

    def control(self,ref):

        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value

        e = self.tilt_angle - ref
        ed = self.tilt_angle_vel
        self.ei += self.Ts*e

        u = -self.kp*e - self.kd*ed - self.ki*self.ei
        u = np.clip(u,-1.0,1.0)
        print(f"u:{u}")
        if np.isfinite(u):
            print(f"u:{u}")
            self.spin_motor(u)
        else:
            u = 0.0
        return u
   
    def on_shutdown(self):
        self.stop()
        self.rc._port.close()
        self.get_logger().info("port closed !")


def main(args=None):
    try:
        rclpy.init(args=args)
        tilt_controller = TiltController()
        tilt_controller.get_logger().info("Starting tilt controller node...")
        rclpy.spin(tilt_controller)
    except KeyboardInterrupt:
        pass
    finally:
        tilt_controller.on_shutdown()  # do any custom cleanup
        tilt_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()