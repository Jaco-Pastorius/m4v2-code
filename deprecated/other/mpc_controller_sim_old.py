# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import TiltAngle

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleThrustSetpoint

from custom_msgs.msg import MPCStatus

# Acados imports
from acados_template import AcadosOcpSolver

# Python imports
import numpy as np
from numpy import cos,sin
import time 

# Morphing lander imports
from morphing_lander.morphing_lander_dynamics import S_numeric
from morphing_lander.morphing_lander_mpc import create_ocp_solver_description
from morphing_lander.trajectories import traj_jump_time, traj_circle_time, emergency_descent_time
from morphing_lander.parameters import params_

# Get parameters
queue_size                 = params_.get('queue_size')
N_horizon                  = params_.get('N_horizon')
Ts                         = params_.get('Ts')
v_max_absolute             = params_.get('v_max_absolute')
u_max                      = params_.get('u_max')
land_height                = params_.get('land_height')
warmup_time                = params_.get('warmup_time')
m                          = params_.get('m')
g                          = params_.get('g')
T_max                      = params_.get('T_max')
emergency_descent_velocity = params_.get('emergency_descent_velocity')

class OffboardControl(Node): 
    def __init__(self):
        super().__init__('OffboardControl')

        # publishers
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", queue_size)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", queue_size)
        self.actuator_motors_publisher_   = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", queue_size)
        self.trajectory_setpoint_publisher_   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", queue_size)
        self.vehicle_rates_setpoint_publisher_   = self.create_publisher(VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", queue_size)
        self.vehicle_attitude_setpoint_publisher_   = self.create_publisher(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", queue_size)
        self.tilt_angle_publisher_   = self.create_publisher(TiltAngle, "fmu/in/tilt_angle", queue_size)
        self.mpc_status_publisher_   = self.create_publisher(MPCStatus, "/mpc_status", queue_size)
        self.vehicle_thrust_setpoint_publisher_   = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", queue_size)

        # subscribers
        # self.vehicle_odometry_subscriber = self.create_subscription(
        #                                     VehicleOdometry,
        #                                     '/fmu/out/vehicle_odometry',
        #                                     self.vehicle_odometry_callback,
        #                                     qos_profile_sensor_data)
        # self.vehicle_odometry_subscriber  # prevent unused variable warning

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

        # timer callback
        self.timer_ = self.create_timer(Ts, self.timer_callback)

        # offboard mode
        self.offboard_setpoint_counter_ = 0
        self.offboard = False

        # if true, offboard mode is exited and goes to px4 failsafe
        self.mpc_failed = False

        # emergency descent
        self.set_emergency_xyz_trajectory = False 
        self.emergency_xyz_position = np.zeros(3,dtype='float')
        self.emergency_t0 = 0.0

        # robot state
        self.state = np.zeros(12)

        # tilt angle
        self.tilt_angle = 0.0

        # time variables
        self.time_initialized = False
        self.initial_time = 0.0
        self.current_time = 0.0
        self.dt = 0.0

        # counter
        self.counter = 0

        # acados solver
        self.ocp = create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_ocp_" + self.ocp.model.name + ".json"
        )

        # solver initialization
        for stage in range(N_horizon + 1):
            self.acados_ocp_solver.set(stage, "x", np.zeros(12))
            self.acados_ocp_solver.set(stage, "p", np.array([0.0]))

        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros(4))

    def timer_callback(self):
        # trigger offboard mode (sets self.offboard) and keep publishing heartbeat
        self.offboard_mode_trigger()

        if self.offboard:
            # trigger mpc after warmup time
            mpc_flag = self.mpc_trigger()
            print(f"mpc_flag: {mpc_flag}")
            print(f"self.tilt_angle: {self.tilt_angle}")

            # when mpc is triggered initialize the time
            if mpc_flag and (not self.time_initialized): self.initialize_time(mpc_flag)
            
            # as long as node is alive continue advancing time
            if self.time_initialized: self.advance_time()

            # get current state and tilt angle (self.state is updated by odometry callback)
            x_current   = np.copy(self.state) 
            phi_current = np.copy(self.tilt_angle)

            # check if robot is grounded
            grounded_flag = self.ground_detector(x_current)

            # update reference (make sure reference starts from ground and ends on ground)
            x_ref,u_ref,tilt_vel,tracking_done = traj_jump_time(self.current_time)

            if tracking_done and grounded_flag:
                print("tracking done and grounded")
                # this means we are on the ground and have already executed the trajectory
                # we need to switch off the thrusters and get to drive as fast as possible (if not there already)
                # also stop running the mpc
                u_opt = np.zeros(4,dtype='float')
                tilt_vel = u_max
                mpc_flag = False      
                self.disarm()
            
            if tracking_done and (not grounded_flag):
                print("tracking done but not grounded")
                # this means we are in the air even after the trajectory has ended
                # this could happen because the trajectory finishes in the air 
                # it could also happen if the tracking has gone wrong
                # in both cases we need to land the vehicle
                # strategy: override reference with a trajectory that tracks to the ground at the current (x,y) position
                # also set u_ref to gravity compensation and tilt_vel to -1.0 (i.e. go as close as possible to drone configuration)
                if not self.set_emergency_xyz_trajectory:
                    self.emergency_t0                 = np.copy(self.current_time)
                    self.emergency_xyz_position       = np.copy(self.state[:3])
                    self.set_emergency_xyz_trajectory = True
                x_ref,u_ref,tilt_vel = emergency_descent_time(self.current_time - self.emergency_t0,self.emergency_xyz_position)
                
                # publish trajectory setpoint for land detector
                msg = TrajectorySetpoint()
                msg.velocity = [0.0,0.0,emergency_descent_velocity]
                self.trajectory_setpoint_publisher_.publish(msg)

            # run mpc
            mpc_status = 0
            if mpc_flag: 
                u_opt,mpc_status,comp_time = self.mpc_update(x_current,phi_current,x_ref,u_ref)
                if mpc_status != 0 :
                    # if mpc fails setting this flag now makes controller 
                    # stop publishing offboard messages and switches to px4 failsafe.
                    # On the real hardware tilt_vel should be set to -1 so that it goes back to drone mode as quickly as possible
                    print("mpc failed !")
                    self.mpc_failed = True
                    return
            else:
                u_opt = np.zeros(4,dtype='float')
                comp_time = -1.0

            # publish control inputs
            self.publish_actuator_motors(u_opt,tilt_vel,grounded_flag)
            msg = VehicleThrustSetpoint()
            msg.xyz = [0.0,0.0,-np.sum(u_opt)/4]
            self.vehicle_thrust_setpoint_publisher_ .publish(msg)

            # publish log values
            self.publish_log(x_current,u_opt,x_ref,u_ref,self.tilt_angle,tilt_vel,mpc_status,comp_time)

    def vehicle_odometry_callback(self, msg): 
        # get state from odometry
        p = msg.position
        phi,th,psi = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity

        self.state = np.array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2]])
        self.q = np.array([p[0],p[1],p[2],psi,th,phi])
        self.u = np.array([v[0],v[1],v[2],o[0],o[1],o[2]])

    def vehicle_local_position_groundtruth_callback(self, msg): 
        self.state[0] = msg.x
        self.state[1] = msg.y
        self.state[2] = msg.z
        self.state[6] = msg.vx
        self.state[7] = msg.vy
        self.state[8] = msg.vz

    def vehicle_attitude_groundtruth_callback(self, msg): 
        phi,th,psi = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        self.state[3] = psi
        self.state[4] = th
        self.state[5] = phi

    def vehicle_angular_velocity_groundtruth_callback(self, msg): 
        self.state[9]  = msg.xyz[0]
        self.state[10] = msg.xyz[1]
        self.state[11] = msg.xyz[2]

    def mpc_update(self,xcurrent,phicurrent,x_ref,u_ref):
        start_time = time.process_time()

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", xcurrent)
        self.acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref and parameters
        for j in range(N_horizon):
            yref = np.hstack((x_ref,u_ref))
            self.acados_ocp_solver.set(j, "yref", yref)
            self.acados_ocp_solver.set(j, "p", np.array([phicurrent]))
        yref_N = x_ref
        self.acados_ocp_solver.set(N_horizon, "yref", yref_N)
        self.acados_ocp_solver.set(N_horizon, "p", np.array([phicurrent]))

        # solve ocp
        mpc_status = self.acados_ocp_solver.solve()  
        print(f"mpc_status: {mpc_status}")

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")      

        comp_time = time.process_time() - start_time
        print("* comp time = %5g seconds\n" % (comp_time))

        return u_opt, mpc_status, comp_time

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    def ground_detector(self,x_current):
        grounded_flag = False
        if x_current[2]>land_height:
            grounded_flag = True
        return grounded_flag

    def mpc_trigger(self):
        mpc_flag = False 
        self.counter += 1
        if (self.counter > int(warmup_time/Ts)): 
            mpc_flag = True
        return mpc_flag
    
    def initialize_time(self,mpc_flag):
        self.initial_time = Clock().now().nanoseconds/1e9
        self.previous_time = self.initial_time
        self.dt = 0.0
        self.time_initialized = True

    def advance_time(self):
        if self.time_initialized:
            self.current_time = Clock().now().nanoseconds/1e9 - self.initial_time
            self.dt = self.current_time - self.previous_time
            self.previous_time = self.current_time

    def offboard_mode_trigger(self):
        # go to offboard mode after 10 setpoints and stop counter after reaching 11 
        if (self.offboard_setpoint_counter_ == 10):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
            self.offboard = True
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

        # px4 needs to receive offboard mode messages so that it will arm
        # once it arms and self.offboard becomes true then the heartbeat is kept alive in the timer callback
        if (not self.mpc_failed) : self.publish_offboard_control_mode_direct_actuator() 

    def publish_offboard_control_mode_direct_actuator(self):
        msg = OffboardControlMode()
        msg.position = False  
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_offboard_control_mode_body_rate(self):
        msg = OffboardControlMode()
        msg.position = False  
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_offboard_control_mode_position(self):
        msg = OffboardControlMode()
        msg.position = True  
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_offboard_control_mode_attitude(self):
        msg = OffboardControlMode()
        msg.position = False  
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_actuator_motors(self,u,tilt_vel,grounded_flag):
        # update tilt angle
        self.tilt_angle += self.dt * tilt_vel * v_max_absolute
        if grounded_flag:
            self.tilt_angle = np.clip(self.tilt_angle,0,np.deg2rad(85))
        else:
            self.tilt_angle = np.clip(self.tilt_angle,0,np.deg2rad(50))
        self.publish_tilt_angle(self.tilt_angle)
        
        # publish to px4
        print(f"publishing: {u}")
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]
        msg.control[4] = self.tilt_angle/(np.pi/2)
        msg.control[5] = self.tilt_angle/(np.pi/2)

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.actuator_motors_publisher_.publish(msg)

    def publish_vehicle_rates(self,w_opt):
        print("publishing")
        msg = VehicleRatesSetpoint()
        msg.roll  = w_opt[0]
        msg.pitch = w_opt[1]
        msg.yaw   = w_opt[2]
        msg.thrust_body = [0.0,0.0,w_opt[3]]

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.vehicle_rates_setpoint_publisher_.publish(msg)

    def publish_vehicle_attitude_setpoint(self,att_opt):
        q = self.quaternion_from_euler(att_opt[0],att_opt[1],att_opt[2])

        msg = VehicleAttitudeSetpoint()
        msg.q_d[0] = q[0]
        msg.q_d[1] = q[1]
        msg.q_d[2] = q[2]
        msg.q_d[3] = q[3]

        msg.thrust_body =  [0.0,0.0,att_opt[3]]
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_attitude_setpoint_publisher_.publish(msg)   

    def publish_tilt_angle(self,tilt_angle):
        msg = TiltAngle()
        msg.value = tilt_angle
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.tilt_angle_publisher_.publish(msg)

    def publish_trajectory_setpoint(self,pos):
        msg = TrajectorySetpoint()
        msg.position = pos
        msg.yaw = 0.0 # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def publish_log(self,x,u,x_ref,u_ref,tilt_angle,tilt_vel,status,comptime):
        msg = MPCStatus()
        msg.status = status
        msg.comptime = comptime
        for i in range(4):
            msg.input[i]  = u[i]
        msg.x = x[0]
        msg.y = x[1]
        msg.z = x[2]
        msg.thetaz = x[3]
        msg.thetay = x[4]
        msg.thetax = x[5]
        msg.dx = x[6]
        msg.dy = x[7]
        msg.dz = x[8]
        msg.omegax = x[9]
        msg.omegay = x[10]
        msg.omegaz = x[11]

        msg.varphi  = tilt_angle
        msg.tiltvel = tilt_vel

        msg.xref = x_ref.astype('float32')
        msg.uref = u_ref.astype('float32')

        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.mpc_status_publisher_.publish(msg)

    def euler_from_quaternion(self,quaternion):
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
    
    def quaternion_from_euler(self,phi,th,psi):
        w = cos(phi/2)*cos(th/2)*cos(psi/2) + sin(phi/2)*sin(th/2)*sin(psi/2)
        x = sin(phi/2)*cos(th/2)*cos(psi/2) - cos(phi/2)*sin(th/2)*sin(psi/2)
        y = cos(phi/2)*sin(th/2)*cos(psi/2) + sin(phi/2)*cos(th/2)*sin(psi/2)
        z = cos(phi/2)*cos(th/2)*sin(psi/2) - sin(phi/2)*sin(th/2)*cos(psi/2)
        return np.array([w,x,y,z])

    def u_to_w(self,acados_ocp_solver,tilt_angle):
        # body rate control conversion (unused)
        u_opt = acados_ocp_solver.get(0, "u")
        x_opt = acados_ocp_solver.get(1, "x")  
        c = (S_numeric(np.zeros(12),tilt_angle).T @ u_opt)[2]/T_max
        omega_d = [x_opt[9],x_opt[10],x_opt[11]]
        w_opt = omega_d + [c]
        return w_opt

def main(args=None):
    rclpy.init(args=args)
    print("Starting model predictive controller...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
