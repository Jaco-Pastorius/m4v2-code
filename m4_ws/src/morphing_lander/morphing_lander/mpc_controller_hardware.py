# Numpy imports
from numpy import array

# ROS imports
import rclpy
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import InputRc
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorMotors

# Morphing lander imports
from morphing_lander.mpc.MPCBase    import MPCBase
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.utils      import euler_from_quaternion

min          = params_.get('min')
max          = params_.get('max')
dead         = params_.get('dead')

class MPCHardware(MPCBase): 
    def __init__(self):
        super().__init__()

        # subscribers
        self.rc_subscription = self.create_subscription(
                                    InputRc,
                                    '/fmu/out/input_rc',
                                    self.rc_listener_callback,
                                    qos_profile_sensor_data)
        self.rc_subscription         
        self.vehicle_odometry_subscriber = self.create_subscription(
                                            VehicleOdometry,
                                            '/fmu/out/vehicle_odometry',
                                            self.vehicle_odometry_callback,
                                            qos_profile_sensor_data)
        self.vehicle_odometry_subscriber  

        self.min  = min
        self.max  = max
        self.dead = dead

        self.offboard_switch = False
        self.mpc_switch      = False

    def mpc_trigger(self):
        return self.mpc_switch
        
    def offboard_mode_trigger(self):
        return self.offboard_switch
    
    def get_reference(self):
        pass

    def vehicle_odometry_callback(self, msg): 
        # get state from odometry
        p = msg.position
        phi,th,psi = euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity
        self.state = array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2]])

    def rc_listener_callback(self, msg):
        if (msg.values[8] == self.max):
            self.offboard_switch = True
        else:
            self.offboard_switch = False

        if (msg.values[7] == self.max):
            self.mpc_switch = True
        else:
            self.mpc_switch = False

    def publish_actuator_motors(self,u):
        # publish to px4
        print(f"publishing: {u}")
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.actuator_motors_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Spinning MPCHardware node... \n")
    mpc_hardware = MPCHardware()
    rclpy.spin(mpc_hardware)
    mpc_hardware.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
