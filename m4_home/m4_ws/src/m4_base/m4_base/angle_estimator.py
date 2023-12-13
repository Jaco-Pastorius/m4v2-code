import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        self.subscription = self.create_subscription(
            Float32,
            'tilt_angle',
            self.tilt_angle_callback,
            10
        )
        self.publisher = self.create_publisher(Float32, 'tilt_angle_filtered', 10)

        # Initialize Kalman filter parameters
        self.angle_estimate = 0.0
        self.angle_covariance = 1.0
        self.process_noise = 0.1
        self.measurement_noise = 0.5

    def tilt_angle_callback(self, msg):
        # Apply Kalman filter to the tilt angle
        predicted_estimate = self.angle_estimate
        predicted_covariance = self.angle_covariance + self.process_noise

        kalman_gain = predicted_covariance / (predicted_covariance + self.measurement_noise)
        self.angle_estimate = predicted_estimate + kalman_gain * (msg.data - predicted_estimate)
        self.angle_covariance = (1 - kalman_gain) * predicted_covariance

        # Publish the filtered tilt angle
        filtered_msg = Float32()
        filtered_msg.data = self.angle_estimate
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)

    kalman_filter_node = KalmanFilterNode()

    try:
        rclpy.spin(kalman_filter_node)
    except KeyboardInterrupt:
        pass

    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
