# Numpy imports
from numpy import array,zeros

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

min            = params_.get('min')
max            = params_.get('max')
dead           = params_.get('dead')
max_dx         = params_.get('max_dx')
max_dy         = params_.get('max_dy')
max_dz         = params_.get('max_dz')
tilt_height    = params_.get('tilt_height')

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

        # control input
        self.input       = zeros(4) # vx, vy, vz, x

    def mpc_trigger(self):
        return self.mpc_switch
        
    def offboard_mode_trigger(self):
        return self.offboard_switch
    
    def get_reference(self):
        drive_vel = [0.0,0.0]
        x_ref = zeros(12)
        u_ref = zeros(4)

        x_ref[0] = self.state[0] + self.input[0]
        x_ref[1] = self.state[1] + self.input[1]
        x_ref[2] = self.state[2] + self.input[2]

        x_ref[6] = self.input[0]
        x_ref[7] = self.input[1]
        x_ref[8] = self.input[2]

        if not self.mission_done:
            if self.takeoff_flag and abs(self.state[2]) < abs(tilt_height):
                tilt_vel = 1.0
            else:
                tilt_vel = -1.0
            if self.in_transition:
                drive_vel[0] = self.input[0]/max_dx
                drive_vel[1] = self.input[1]/max_dy
        else:
            drive_vel[0] = self.input[0]/max_dx
            drive_vel[1] = self.input[1]/max_dy
            tilt_vel = 1.0

        return x_ref, u_ref, tilt_vel, drive_vel
    
    def vehicle_odometry_callback(self, msg): 
        # get state from odometry
        p = msg.position
        phi,th,psi = euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity
        self.state = array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2]])
        self.state_rl = array([p[0],p[1],p[2],msg.q[1],msg.q[2],msg.q[3],msg.q[0],v[0],v[1],v[2],o[0],o[1],o[2]])

    def rc_listener_callback(self, msg):
        if (msg.values[8] == self.max):
            self.offboard_switch = True
        else:
            self.offboard_switch = False

        if (msg.values[7] == self.max):
            self.mpc_switch = True
        else:
            self.mpc_switch = False

        # get input for manual control
        self.input[0]  = max_dx * -(msg.values[1]-dead)/(max-dead)
        self.input[1]  = max_dy *  (msg.values[0]-dead)/(max-dead)
        self.input[2]  = max_dz * -(msg.values[2]-dead)/(max-dead)

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
