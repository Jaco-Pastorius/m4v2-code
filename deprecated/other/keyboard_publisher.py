import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import tty
import termios

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Float64, '/tilt_angle', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.value = 0.0
        self.step_size = 0.05
        self.max_value = 1.0
        self.min_value = -1.0

    def timer_callback(self):
        self.run()
        msg = Float64()
        msg.data = self.value
        self.publisher_.publish(msg)

    def increase_value(self):
        self.value = min(self.value + self.step_size, self.max_value)
        print(self.value)

    def decrease_value(self):
        self.value = max(self.value - self.step_size, self.min_value)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        key = self.get_key()
        if key == 'w':
            self.increase_value()
        elif key == 'x':
            self.decrease_value()

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()