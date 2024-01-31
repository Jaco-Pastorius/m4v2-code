import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry  # Import the custom message

class MocapToVisualOdometry(Node):

    def __init__(self):
        super().__init__('mocap_to_visual_odometry')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/m4_base/pose',
            self.mocap_pose_callback,
            10)
        self.publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10)

    def mocap_pose_callback(self, msg):
        # Process the PoseStamped message and publish to /fmu/in/vehicle_visual_odometry

        vehicle_odom_msg = VehicleOdometry()

        # Set the timestamp
        vehicle_odom_msg.timestamp = msg.header.stamp.to_msg().sec * 1e6 + msg.header.stamp.to_msg().nanosec / 1e3

        # Set the pose_frame (assuming NED for simplicity)
        # vehicle_odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # Set position
        vehicle_odom_msg.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        # Set orientation (using quaternion)
        vehicle_odom_msg.q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

        # Set velocity_frame (assuming NED for simplicity)
        # vehicle_odom_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # Set velocity, angular_velocity, and other fields as needed

        # Publish the transformed message
        self.publisher.publish(vehicle_odom_msg)

def main(args=None):
    rclpy.init(args=args)

    mocap_to_visual_odometry_node = MocapToVisualOdometry()

    rclpy.spin(mocap_to_visual_odometry_node)

    mocap_to_visual_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
