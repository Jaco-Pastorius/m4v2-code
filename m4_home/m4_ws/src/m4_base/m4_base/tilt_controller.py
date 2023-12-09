import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc
from std_msgs.msg import Int32

# roboclaw and jetson
from m4_base.roboclaw_3 import Roboclaw

class TiltController(Node):
    def __init__(self):
        super().__init__('tilt_controller')

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        # self.timer_ = self.create_timer(self.timer_period, self.timer_callback_encoder_counts)

        self.tilt_angle_min = 1094 # corresponds to zero degrees i.e. fly configuration
        self.tilt_angle_max = 1934 # corresponds to 90 degrees i.e. drive configuration
        self.tilt_angle_dead = 1514 # corresponds to midway through i.e. 45 degrees

        self.drive_speed_dead = 1514
        self.drive_speed_min = 1934
        self.drive_speed_max = 1094

        self.tilt_speed = 0.0 # fly configuration
        self.encoder_counts = 0
        self.LS_in = 1514

        # roboclaw stuff
        self.address = 0x80
        self.rc = Roboclaw("/dev/ttyACM0",115200)
        self.rc.Open()

    def listener_callback(self, msg):
        # https://futabausa.com/wp-content/uploads/2018/09/18SZ.pdf
        self.LS_in = msg.values[9] # corresponds to LS trim selector on futaba T18SZ that I configured in the function menu
        self.get_logger().info("LS_in: {}".format(self.LS_in))
        # self.get_logger().info("drive_speed_in: {}".format(self.drive_speed_in))

    def timer_callback(self):
        self.tilt_speed = self.normalize(self.LS_in)
        motor_speed = self.map_speed(abs(self.tilt_speed))
        if (self.tilt_speed < 0): # go up
            if motor_speed == 127: 
                motor_speed = 126 # weird bug not sure why this is needed
            self.rc.BackwardM2(self.address, motor_speed)
        else:
            self.rc.ForwardM2(self.address, motor_speed)
        # else:
            # if tilt speed is zero do a position hold
            # self.rc.SetEncM1(self.address,0)
            # position = 0
            # self.rc.SpeedAccelDeccelPositionM1(self.address,1000,1000,1000,position,1)

    def timer_callback_encoder_counts(self):
        self.rc.SetEncM2(self.address,0)
        self.encoder_counts = self.normalize(self.LS_in)
        distance = self.map_encoder_counts(abs(self.encoder_counts))
        if (self.encoder_counts < 0): # go up
            self.rc.SpeedDistanceM2(self.address,1000,-distance,1)
        else :
            self.rc.SpeedDistanceM2(self.address,1000,distance,1)

    def normalize(self,LS_in):
        return (LS_in-self.tilt_angle_dead)/(self.tilt_angle_max-self.tilt_angle_dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def map_encoder_counts(self,speed_normalized):
        return int(1000*speed_normalized)

    def stop(self):
        self.rc.ForwardM2(self.address,self.map_speed(0.0))

    def publish_drive_speed(self):
        msg = Int32()
        msg.data = int(self.drive_speed_in)
        self.drive_speed_publisher_.publish(msg)

    # def goDrive():
        # self.rc.ForwardM1(self.address,map_speed(0.25))

    # def goFly():
        # self.rc.BackwardM1(self.address,map_speed(0.35))

    # def callback_down(channel):
        # self.stop()
        
    # def callback_up(channel):
        # print("limswitch up pressed !")

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