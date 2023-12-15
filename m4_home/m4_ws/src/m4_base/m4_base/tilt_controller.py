import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc
from std_msgs.msg import Int32, Float32
from copy import deepcopy

import numpy as np

# roboclaw and jetson
from m4_base.roboclaw_3 import Roboclaw

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

        # Tilt angle subscription
        self.angle_subscription = self.create_subscription(
            Float32,
            '/tilt_angle',
            self.angle_listener_callback,
            qos_profile_sensor_data)
        self.angle_subscription  # prevent unused variable warning

        # Publisher filtered
        self.publisher_ref = self.create_publisher(Float32, 'tilt_angle_ref', 10)
        self.publisher_filtered = self.create_publisher(Float32, 'tilt_angle_filtered', 10)
        self.publisher_vel = self.create_publisher(Float32, 'tilt_angle_vel', 10)

        # Timer
        self.Ts = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set min, max, and dead values for LS_in (the control for the tilt angle coming from rc input)
        self.LS_min = 1094 # corresponds to zero degrees i.e. fly configuration
        self.LS_max = 1934 # corresponds to 90 degrees i.e. drive configuration
        self.LS_dead = 1514 # corresponds to midway through i.e. 45 degrees

        # Set min, max, and dead values for manual_in
        self.manual_in_min = 1094
        self.manual_in_max = 1934
        self.manual_in_dead = 1514

        # Set min and max for ref_angle
        self.ref_angle_min = 1094
        self.ref_angle_max = 1934

        # Initialize LS_in
        self.LS_in = 1514

        # Initialize tilt_angle_in
        self.tilt_angle_in = 0.0
        self.prev_tilt_angle_in = 0.0

        self.tilt_angle_vel = 0.0
        self.prev_estimate = 0.0

        # Manual vs automatic control
        self.manual = True

        # Set temporary tilt_angle reference
        self.tilt_angle_ref = 0.0

        # Roboclaw
        self.address = 0x80
        self.rc = Roboclaw("/dev/ttyACM1",115200)
        self.rc.Open()

        # Initialize kalman filter
        self.xkk = 0.0
        self.pkk = 1.0
        self.Rv,self.Rw = 1.0,5.0
        self.xkkm,self.pkkm = None, None

    def rc_listener_callback(self, msg):
        # https://futabausa.com/wp-content/uploads/2018/09/18SZ.pdf
        self.LS_in = msg.values[9] # corresponds to LS trim selector on futaba T18SZ that I configured in the function menu
        self.tilt_angle_ref = self.map_ref_angle(msg.values[2])

        # set manual or automatic control of tilt angle
        if msg.values[7] == 1934:
            self.manual = False
            print("automatic tracking ! ")
        else:
            self.manual = True

        msg = Float32()
        msg.data = self.tilt_angle_ref
        self.publisher_ref.publish(msg)

        # self.get_logger().info("LS_in: {}".format(self.LS_in))
        # self.get_logger().info("manual: {}".format(self.manual))

    def angle_listener_callback(self, msg):
        self.prev_tilt_angle_in = deepcopy(self.tilt_angle_in)
        self.tilt_angle_in = msg.data # in degrees
        # self.get_logger().info("tilt_angle_in: {}".format(self.tilt_angle_in))

    def timer_callback(self):

        # compute tilt_angle velocity
        if ((self.xkkm is not None) and (self.prev_estimate is not None)):
            self.tilt_angle_vel = (self.xkkm - self.prev_estimate)/self.Ts

        msg = Float32()
        msg.data = self.tilt_angle_vel
        self.publisher_vel.publish(msg)

        if (self.manual):
            # manual control of tilt angle
            u = self.normalize(self.LS_in)
            self.spin_motor(u)
            self.estimate(u,self.tilt_angle_in)
        else:
            # automatic control of tilt angle
            u = self.control()
            self.estimate(u,self.tilt_angle_in)
  
    def normalize(self,LS_in):
        return (LS_in-self.LS_dead)/(self.LS_max-self.LS_dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)
    
    def map_ref_angle(self,ref_angle):
        return 90*(ref_angle - self.ref_angle_min)/(self.ref_angle_max - self.ref_angle_min) # in degrees

    def stop(self):
        self.rc.ForwardM2(self.address,self.map_speed(0.0))

    def spin_motor(self, tilt_speed):
        # takes in a tilt_speed between -1 and 1 writes the pwm signal to the roboclaw 
        motor_speed = self.map_speed(abs(tilt_speed))
        if (tilt_speed < 0): # go up
            if motor_speed == 127: 
                motor_speed = 126 # weird bug not sure why this is needed
            self.rc.ForwardM2(self.address, motor_speed)
        else:
            self.rc.BackwardM2(self.address, motor_speed)

    def estimate(self,uk,yk):

        self.prev_estimate = deepcopy(self.xkkm)

        # Run kalman filter
        self.predict(uk)
        self.correct(yk)

        msg = Float32()
        msg.data = self.xkkm
        self.publisher_filtered.publish(msg)

    def predict(self,uk):
        self.xkkm = self.xkk + self.Ts * uk
        self.pkkm = self.pkk + self.Rv

    def correct(self,y):
        l = self.pkkm * 1/(self.Rw + self.pkkm)
        self.xkk = self.xkkm - l*(self.xkkm - y)
        self.pkk = self.pkkm - l*self.pkkm

    def control(self):
        kp, kd = 0.05, 0.01
        e = self.xkkm - self.tilt_angle_ref
        ed = self.tilt_angle_vel
        u = -kp*e - kd*ed
        print(f"u:{u}")
        if np.isfinite(u):
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
        tilt_controller.get_logger().info("Starting rc_subscriber...")
        rclpy.spin(tilt_controller)
    except KeyboardInterrupt:
        pass
    finally:
        tilt_controller.on_shutdown()  # do any custom cleanup
        tilt_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()