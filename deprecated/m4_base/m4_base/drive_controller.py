import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc
from std_msgs.msg import Int32

# roboclaw and jetson
from m4_base.roboclaw_3 import Roboclaw

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

        self.drive_speed_dead = 1514
        self.drive_speed_min = 1934
        self.drive_speed_max = 1094

        # initialize drive speed
        self.drive_speed_in = 1514

        # roboclaw stuff
        self.address = 0x80
        self.rc = Roboclaw("/dev/ttyACM0",115200)
        self.rc.Open()

    def listener_callback(self, msg):
        self.drive_speed_in  = msg.values[1]
        self.turn_speed_in  = msg.values[0]

    def timer_callback(self):
        lin_vel = self.map_speed(self.normalize(self.drive_speed_in))
        ang_vel = self.map_speed(self.normalize(self.turn_speed_in))

        self.get_logger().info(f"hello: lin_vel, ang_vel: ({lin_vel},{ang_vel})")

        self.move_right_wheel(lin_vel + ang_vel)
        self.move_left_wheel(lin_vel - ang_vel)

    def normalize(self,drive_speed_in):
        return (drive_speed_in-self.drive_speed_dead)/(self.drive_speed_max-self.drive_speed_dead)

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def on_shutdown(self):
        self.rc._port.close()
        self.get_logger().info("port closed !")

    def move_right_wheel(self, speed):
        if speed > 0:
            if speed >= 127:
                speed = 126
            self.rc.BackwardM1(self.address, abs(speed))
        else:
            if speed <= -127:
                speed = -126
            self.rc.ForwardM1(self.address, abs(speed))


    def move_left_wheel(self, speed):
        if speed > 0:
            if speed >= 127:
                speed = 126
            self.rc.ForwardM2(self.address, abs(speed))
        else:
            if speed <= -127:
                speed = -126
            self.rc.BackwardM2(self.address, abs(speed))


def main(args=None):
    try:
        rclpy.init(args=args)
        drive_controller = DriveController()
        drive_controller.get_logger().info("Starting drive_controller...")
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drive_controller.on_shutdown()  # do any custom cleanup
        drive_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()