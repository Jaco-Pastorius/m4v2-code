import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc, TiltAngle
from std_msgs.msg import Int32, Float32
from copy import deepcopy

import numpy as np

from scipy.interpolate import interp1d

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

        # External tilt angle ref subscription
        self.tilt_angle_ref_external_subscription = self.create_subscription(
            Float32,
            '/tilt_angle_ref_external',
            self.tilt_angle_ref_external_listener_callback,
            qos_profile_sensor_data)
        self.tilt_angle_ref_external_subscription  # prevent unused variable warning

        # Publisher tilt angle
        self.publisher_tilt = self.create_publisher(TiltAngle, '/fmu/in/tilt_angle', 10)

        # Publisher filtered
        self.publisher_ref = self.create_publisher(Float32, 'tilt_angle_ref', 10)
        self.publisher_filtered = self.create_publisher(Float32, 'tilt_angle_filtered', 10)
        self.publisher_vel = self.create_publisher(Float32, 'tilt_angle_vel', 10)

        # Timer
        self.Ts = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Set min, max, and dead values for LS_in (the control for the tilt angle coming from rc input)
        self.min = 1094 # corresponds to zero degrees i.e. fly configuration
        self.max = 1934 # corresponds to 90 degrees i.e. drive configuration
        self.dead = 1514 # corresponds to midway through i.e. 45 degrees

        # Initialize LS_in
        self.LS_in = 1514

        # Initialize tilt_angle_in
        self.tilt_angle_in = 0.0
        self.prev_tilt_angle_in = 0.0

        self.tilt_angle_vel = 0.0
        self.prev_estimate = 0.0

        # Manual vs automatic control
        self.manual = True
        self.external_ref = False

        # Set temporary tilt_angle reference
        self.tilt_angle_ref = 0.0
        self.tilt_angle_ref_external = 0.0

        # Roboclaw
        self.address = 0x80
        self.rc = Roboclaw("/dev/ttyACM1",115200)
        self.rc.Open()

        # set encoder count to zero (always start robot in drive configuration)
        self.rc.SetEncM2(self.address,0)
        self.reset_encoder = 0

        # set pin functions for motor 2 (M2) to go to zero when it reaches home
        self.rc.SetPinFunctions(self.address,0x00,0x62,0x62)

        # tabulate encoder vs angle
        # self.encoder_data = np.array([0,2518,4902,6813,8429,9396,
        #                               10477,11156,13324,14293,
        #                               15396,16820,17563,19043,
        #                               16677,14629,12845,11206,8887,
        #                               6007,3631,1424,2680,4408,
        #                               5960,6976,8223,9351,
        #                               10567,11136,12376,13456,14391,
        #                               16464,17294,17662,19046,
        #                               17648,16080,13721,12225,10929,9137,
        #                               7513,6009,4345,2929,1489])
        
        # self.angle_data = np.array([88,85,78.4,70.7,62.7,57.3,51.2,47.1,33.8,27.9,21.3,12.7,8.3,0.5,
        #                             13.4,25.8,36.6,46.7,60.2,74.2,82.4,86.7,84.6,80.1,
        #                             74.4,70.0,63.7,57.6,50.6,47.1,39.7,32.8,27.2,14.6,9.8,7.6,0.6,7.5,16.8,
        #                             31.2,40.4,48.3,58.6,67.3,74.1,80.3,84.1,86.7])
        
        self.encoder_data = np.array([0,2518,4902,6813,8429,9396,
                                      10477,11156,13324,14293,
                                      15396,16820,17563,19043])
        
        self.angle_data = np.array([88,85,78.4,70.7,62.7,57.3,51.2,47.1,33.8,27.9,21.3,12.7,8.3,0.5])

        # idx_sort = np.argsort(self.encoder_data)
        # self.encoder_data = self.encoder_data[idx_sort]
        # self.angle_data = self.angle_data[idx_sort]
        
        self.n_phi = lambda phi : np.interp(phi,self.angle_data,self.encoder_data)

        # Initialize kalman filter
        self.xkk = 0.0
        self.pkk = 1.0
        self.Rv,self.Rw = 1.0,5.0
        self.xkkm,self.pkkm = None, None

        # Declare parameters
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

        # initialize integral error
        self.ei = 0.0

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

        # set external reference tracking 
        if msg.values[8] == self.max:
            self.external_ref = True
        else:
            self.external_ref = False

        self.tilt_angle_ref = self.map_ref_angle(msg.values[2])
        msg = Float32()
        msg.data = self.tilt_angle_ref
        self.publisher_ref.publish(msg)

        # self.get_logger().info("LS_in: {}".format(self.LS_in))
        # self.get_logger().info("manual: {}".format(self.manual))

    def angle_listener_callback(self, msg):
        self.prev_tilt_angle_in = deepcopy(self.tilt_angle_in)
        self.tilt_angle_in = msg.data # in degrees
        # self.get_logger().info("tilt_angle_in: {}".format(self.tilt_angle_in))

    def tilt_angle_ref_external_listener_callback(self, msg):
        self.tilt_angle_ref_external = msg.data # in degrees

    def timer_callback(self):

        if (self.reset_encoder == self.max):
            self.rc.SetEncM2(self.address,0)

        # compute tilt_angle velocity
        if ((self.xkkm is not None) and (self.prev_estimate is not None)):
            self.tilt_angle_vel = (self.xkkm - self.prev_estimate)/self.Ts

        msg = Float32()
        msg.data = self.tilt_angle_vel
        self.publisher_vel.publish(msg)


        # print encoder count
        enc_count = self.rc.ReadEncM2(self.address)

        tilt_angle =  float(np.interp(enc_count[1],self.encoder_data,self.angle_data))
        print(enc_count)
        print(f"enc_count[1] is: {enc_count[1]}")
        print(f"tilt angle is: {tilt_angle}")

        # publish tilt angle
        msg = TiltAngle()
        msg.value = tilt_angle
        self.publisher_tilt.publish(msg)

        if (self.manual):
            # manual control of tilt angle
            u = self.normalize(self.LS_in)
            self.spin_motor(u)
            self.estimate(u,self.tilt_angle_in)

        else:
            # automatic control of tilt angle

            ref_angle = -90 * (self.normalize(self.LS_in) - 1) / 2.0
            print(ref_angle)
            n_ref_angle =  np.interp(ref_angle,self.angle_data,self.encoder_data)
            print(n_ref_angle)
            self.position_control(n_ref_angle)

            # if self.external_ref:
            #     ref = self.tilt_angle_ref_external
            # else:
            #     ref = self.tilt_angle_ref
            # u = self.control(ref)
            # self.estimate(u,self.tilt_angle_in)
  
    def normalize(self,LS_in):
        return (LS_in-self.dead)/(self.max-self.dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)
    
    def map_ref_angle(self,ref_angle):
        return 90*(ref_angle - self.min)/(self.max - self.min) # in degrees

    def stop(self):
        self.rc.ForwardM2(self.address,self.map_speed(0.0))

    def spin_motor(self, tilt_speed):
        # takes in a tilt_speed between -1 and 1 writes the pwm signal to the roboclaw 
        motor_speed = self.map_speed(abs(tilt_speed))
        if (tilt_speed < 0): # go up
            if motor_speed >= 127: 
                motor_speed = 126 # weird bug not sure why this is needed
            self.rc.ForwardM2(self.address, motor_speed)
        else:
            if motor_speed >= 127:
                motor_speed = 126
            self.rc.BackwardM2(self.address, motor_speed)

    def estimate(self,uk,yk):

        self.prev_estimate = deepcopy(self.xkkm)

        # Run kalman filterphi
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

    def control(self,ref):

        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value

        e = self.xkkm - ref
        ed = self.tilt_angle_vel
        self.ei += self.Ts*e

        u = -self.kp*e - self.kd*ed - self.ki*self.ei
        print(f"u:{u}")
        if np.isfinite(u):
            print(f"u:{u}")
            self.spin_motor(u)
        else:
            u = 0.0
        return u
    
    def position_control(self,ref):
        # pass
        self.rc.SpeedAccelDeccelPositionM2(self.address,32000,12000,32000,int(ref),0)

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