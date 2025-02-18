# Numpy imports
from numpy import pi, zeros

# ROS imports
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock

# Message imports
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ManualControlSetpoint

# Morphing lander imports
from morphing_lander.mpc.MPCBase import MPCBase
from morphing_lander.mpc.parameters import params_
from morphing_lander.mpc.utils import euler_from_quaternion
from morphing_lander.mpc.integrator import create_sim_solver_description

warmup_time    = params_['warmup_time']
Ts             = params_['Ts']
v_max_absolute = params_['v_max_absolute']
max_dx         = params_['max_dx']
max_dy         = params_['max_dy']
max_dz         = params_['max_dz']

class MPCBasicSim(MPCBase): 
    def __init__(self):
        super().__init__()

        # subscribers
        self.manual_control_setpoint = self.create_subscription(
                                            ManualControlSetpoint,
                                            '/fmu/out/manual_control_setpoint',
                                            self.manual_control_setpoint_callback,
                                            qos_profile_sensor_data)
        self.manual_control_setpoint  # prevent unused variable warning

        # acados integrator
        sim = create_sim_solver_description()
        self.acados_integrator

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
        x_ref = zeros(12)
        u_ref = zeros(4)

        x_ref[0] = self.state[0] + self.input[0]
        x_ref[1] = self.state[1] + self.input[1]
        x_ref[2] = self.state[2] + self.input[2]

        x_ref[6] = self.input[0]
        x_ref[7] = self.input[1]
        x_ref[8] = self.input[2]

        # if self.input[2] > 0.0 and self.state[2] > -1.0:
        #     tilt_vel = 1.0
        # else:
        #     tilt_vel = -1.0
        tilt_vel = 0.0

        return x_ref, u_ref, tilt_vel

    def publish_actuator_motors(self,u):
        self.acados_integrator.set("u", u)

    def vehicle_local_position_groundtruth_callback(self, msg): 
        self.state[0] = msg.x
        self.state[1] = msg.y
        self.state[2] = msg.z
        self.state[6] = msg.vx
        self.state[7] = msg.vy
        self.state[8] = msg.vz

    def vehicle_attitude_groundtruth_callback(self, msg): 
        phi,th,psi = euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        self.state[3] = psi
        self.state[4] = th
        self.state[5] = phi

    def vehicle_angular_velocity_groundtruth_callback(self, msg): 
        self.state[9]  = msg.xyz[0]
        self.state[10] = msg.xyz[1]
        self.state[11] = msg.xyz[2]

    def manual_control_setpoint_callback(self, msg): 
        self.input[0]  = max_dx * msg.pitch
        self.input[1]  = max_dy * msg.roll
        self.input[2]  = max_dz * -msg.throttle

def main(args=None):
    rclpy.init(args=args)
    print("Spinning MPCBasicSim node \n")
    mpc_sim = MPCBasicSim()
    rclpy.spin(mpc_sim)
    mpc_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
