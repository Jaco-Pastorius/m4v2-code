import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import InputRc, TiltAngle
from custom_msgs.msg import TiltVel
import numpy as np
from morphing_lander.roboclaw_3 import Roboclaw

class TiltVelController(Node):
    def __init__(self):
        super().__init__('tilt_controller')

        # Subscriptions
        self.rc_subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.rc_subscription        # rc subscription
        self.tilt_vel_subscription = self.create_subscription(
            TiltVel,
            '/tilt_vel',
            self.tilt_vel_callback,
            qos_profile_sensor_data)
        self.tilt_vel_subscription  # tilt vel subscription (value between -1 and 1)

        # Publishers
        self.publisher_tilt = self.create_publisher(TiltAngle, '/fmu/in/tilt_angle', 10)

        # Timer
        self.Ts = 0.01 
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Configure RC inputs
        self.min = 1094 
        self.max = 1934 
        self.dead = 1514 

        # Initialize LS_in
        self.LS_in = 1514

        # Manual vs automatic control
        self.manual = True

        # Initialize tilt angle and velocity
        self.tilt_angle = 0.0
        self.tilt_vel   = 0.0

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
        self.encoder_data = np.array([0,3696,5551,7647,8886,10118,11062,11982,12846,13957,14885,15629,16549,17037,17749,19029,19645,20989,21453,22357,22989,23517,24013,24605,25437,25973,26325])
        self.angle_data = np.array([90,86.6,84.0,78.6,74.2,70.5,66.5,63.5,59.7,55.3,51.3,48.0,44.2,41.8,38.4,32.8,30.0,24.2,22.0,18.0,14.8,12.9,10.8,8.2,4.7,2.8,0.8])

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

    def tilt_vel_callback(self, msg):
        self.tilt_vel = msg.value # in rad/s

    def timer_callback(self):
        if (self.reset_encoder == self.max):
            self.rc.SetEncM2(self.address,0)

        # compute and print current tilt angle
        enc_count = self.rc.ReadEncM2(self.address)
        self.tilt_angle =  float(np.interp(enc_count[1],self.encoder_data,self.angle_data))
        print(f"tilt angle is: {self.tilt_angle}")
        # print(f"encoder count is: {enc_count}")

        # publish tilt angle
        self.publish_tilt_angle(self.tilt_angle)

        if (self.manual):
            # manual control of tilt angle
            u = self.normalize(self.LS_in)
            self.spin_motor(u)
        else:
            self.spin_motor(self.tilt_vel)
  
    def publish_tilt_angle(self):
        msg = TiltAngle()
        msg.value = self.tilt_angle
        self.publisher_tilt.publish(msg)

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

    def on_shutdown(self):
        self.stop()
        self.rc._port.close()
        self.get_logger().info("port closed !")

def main(args=None):
    try:
        rclpy.init(args=args)
        tilt_controller = TiltVelController()
        tilt_controller.get_logger().info("Starting tilt velocity controller node...")
        rclpy.spin(tilt_controller)
    except KeyboardInterrupt:
        pass
    finally:
        tilt_controller.on_shutdown()  # do any custom cleanup
        tilt_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()