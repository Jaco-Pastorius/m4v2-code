import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math
import numpy as np 
from copy import deepcopy

def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0.w
    x0 = q0.x
    y0 = q0.y
    z0 = q0.z

    # Extract the values from q1
    w1 = q1.w
    x1 = q1.x
    y1 = q1.y
    z1 = q1.z

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_x, q0q1_y, q0q1_z, q0q1_w])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class TiltAngleCalculator(Node):
    def __init__(self):
        super().__init__('tilt_angle_calculator')
        self.subscription_arm = self.create_subscription(PoseStamped, '/vrpn_mocap/m4_arm/pose', self.arm_callback, 10)
        self.subscription_base = self.create_subscription(PoseStamped, '/vrpn_mocap/m4_base/pose', self.base_callback, 10)
        self.publisher = self.create_publisher(Float32, 'tilt_angle', 10)
        self.arm_pose = None
        self.base_pose = None

    def arm_callback(self, msg):
        self.arm_pose = msg
        self.compute_and_publish_angle()

    def base_callback(self, msg):
        self.base_pose = msg
        self.compute_and_publish_angle()

    def compute_and_publish_angle(self):
        if self.arm_pose is not None and self.base_pose is not None:

            # Get the base and arm quaternions
            q_base = self.base_pose.pose.orientation
            q_arm = self.arm_pose.pose.orientation

            # Get inverse of base quaternion
            q_base_inv = deepcopy(q_base)
            q_base_inv.w = -q_base.w # Negate for inverse

            # Compute relative quaternion bringing base to arm frame
            qr = quaternion_multiply(q_arm, q_base_inv)

            # Get angle 
            roll, pitch, yaw = euler_from_quaternion(qr)
            relative_angle = np.rad2deg(roll)

            # Publish the relative angle to the 'tilt_angle' topic
            msg = Float32()
            msg.data = relative_angle
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tilt_angle_calculator = TiltAngleCalculator()
    rclpy.spin(tilt_angle_calculator)
    tilt_angle_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
