import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from morphing_lander.mpc.DriveControllerBase import DriveControllerBase

class DriveControllerSim(DriveControllerBase):
    def __init__(self):
        super().__init__('DriveControllerSim')

    def update():
        pass


def main(args=None):
    try:
        rclpy.init(args=args)
        drive_controller = DriveControllerSim()
        drive_controller.get_logger().info("Starting drive_controller_sim...")
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drive_controller.on_shutdown()  # do any custom cleanup
        drive_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()