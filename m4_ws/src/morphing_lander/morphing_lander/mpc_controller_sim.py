# Numpy imports
from numpy import pi, zeros

# ROS imports
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock

# Message imports
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import ManualControlSetpoint

# Morphing lander imports
from morphing_lander.mpc.MPCBase import MPCBase
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.utils import euler_from_quaternion, drive_mixer

warmup_time    = params_['warmup_time']
Ts             = params_['Ts']
v_max_absolute = params_['v_max_absolute']
max_dx         = params_['max_dx']
max_dy         = params_['max_dy']
max_dz         = params_['max_dz']
tilt_height    = params_['tilt_height']

class MPCSim(MPCBase): 
    def __init__(self):
        super().__init__()

        # subscribers
        self.vehicle_local_position_groundtruth = self.create_subscription(
                                            VehicleLocalPosition,
                                            '/fmu/out/vehicle_local_position_groundtruth',
                                            self.vehicle_local_position_groundtruth_callback,
                                            qos_profile_sensor_data)
        self.vehicle_local_position_groundtruth  # prevent unused variable warning

        self.vehicle_attitude_groundtruth = self.create_subscription(
                                            VehicleAttitude,
                                            '/fmu/out/vehicle_attitude_groundtruth',
                                            self.vehicle_attitude_groundtruth_callback,
                                            qos_profile_sensor_data)
        self.vehicle_attitude_groundtruth  # prevent unused variable warning

        self.vehicle_angular_velocity_groundtruth = self.create_subscription(
                                            VehicleAngularVelocity,
                                            '/fmu/out/vehicle_angular_velocity_groundtruth',
                                            self.vehicle_angular_velocity_groundtruth_callback,
                                            qos_profile_sensor_data)
        self.vehicle_angular_velocity_groundtruth  # prevent unused variable warning

        self.manual_control_setpoint = self.create_subscription(
                                            ManualControlSetpoint,
                                            '/fmu/out/manual_control_setpoint',
                                            self.manual_control_setpoint_callback,
                                            qos_profile_sensor_data)
        self.manual_control_setpoint  # prevent unused variable warning

        # control input
        self.input = zeros(4) # vx, vy, vz, vtilt

    def mpc_trigger(self):
        mpc_flag = False
        self.counter += 1
        if (self.counter > int(warmup_time/Ts)): 
            mpc_flag = True
        return mpc_flag
        
    def offboard_mode_trigger(self):
        return True
    
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
                drive_vel[0] = self.input[0]
                drive_vel[1] = self.input[1]    
        else:
            drive_vel[0] = self.input[0]
            drive_vel[1] = self.input[1]
            tilt_vel = 1.0

        return x_ref, u_ref, tilt_vel, drive_vel

    def publish_actuator_motors(self,u):
        # publish to px4
        print(f"publishing: {u}")
        msg = ActuatorMotors()

        # publish thruster actions
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]

        # publish tilt action
        msg.control[4] = self.tilt_angle/(pi/2)
        msg.control[5] = self.tilt_angle/(pi/2)

        # get normalized drive actions (m4 sdf is set up for 0 to be negative velocity 0.5 to be zero and 1 to be positive velocity)
        u_left, u_right    = drive_mixer(self.drive_speed,self.turn_speed)
        u_left_normalized  = (u_left + 1.0)/2.0
        u_right_normalized = -(u_right + 1.0)/2.0

        # publish normalized drive actions
        msg.control[6] = u_right_normalized
        msg.control[7] = u_left_normalized
        msg.control[8] = u_left_normalized
        msg.control[9] = u_right_normalized

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.actuator_motors_publisher_.publish(msg)

    def vehicle_local_position_groundtruth_callback(self, msg): 
        self.state[0] = msg.x
        self.state[1] = msg.y
        self.state[2] = msg.z
        self.state[6] = msg.vx
        self.state[7] = msg.vy
        self.state[8] = msg.vz
        self.state_rl[0] = msg.x
        self.state_rl[1] = msg.y
        self.state_rl[2] = msg.z
        self.state_rl[7] = msg.vx
        self.state_rl[8] = msg.vy
        self.state_rl[9] = msg.vz
        
    def vehicle_attitude_groundtruth_callback(self, msg): 
        phi,th,psi = euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        self.state[3] = psi
        self.state[4] = th
        self.state[5] = phi
        self.state_rl[3] = msg.q[1]
        self.state_rl[4] = msg.q[2]
        self.state_rl[5] = msg.q[3]
        self.state_rl[6] = msg.q[0]

    def vehicle_angular_velocity_groundtruth_callback(self, msg): 
        self.state[9]  = msg.xyz[0]
        self.state[10] = msg.xyz[1]
        self.state[11] = msg.xyz[2]
        self.state_rl[10] = msg.xyz[0]
        self.state_rl[11] = msg.xyz[1]
        self.state_rl[12] = msg.xyz[2]

    def manual_control_setpoint_callback(self, msg): 
        self.input[0]  = max_dx * msg.pitch
        self.input[1]  = max_dy * msg.roll
        self.input[2]  = max_dz * -msg.throttle

def main(args=None):
    rclpy.init(args=args)
    print("Spinning MPCSim node \n")
    mpc_sim = MPCSim()
    rclpy.spin(mpc_sim)
    mpc_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
