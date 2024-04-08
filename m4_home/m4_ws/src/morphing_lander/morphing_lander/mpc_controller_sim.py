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

from custom_msgs.msg import MPCStatus

# Python imports
import numpy as np
import time 

# Morphing lander MPC import
from morphing_lander.morphing_lander_mpc import *
from morphing_lander.trajectories import *

QUEUE_SIZE = 1

class OffboardControl(Node): 
    def __init__(self):
        super().__init__('OffboardControl')

        # publishers
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", QUEUE_SIZE)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", QUEUE_SIZE)
        self.actuator_motors_publisher_   = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", QUEUE_SIZE)
        self.trajectory_setpoint_publisher_   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", QUEUE_SIZE)
        self.vehicle_rates_setpoint_publisher_   = self.create_publisher(VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", QUEUE_SIZE)
        self.vehicle_attitude_setpoint_publisher_   = self.create_publisher(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", QUEUE_SIZE)
        self.tilt_angle_publisher_   = self.create_publisher(TiltAngle, "fmu/in/tilt_angle", QUEUE_SIZE)
        self.mpc_status_publisher_   = self.create_publisher(MPCStatus, "/mpc_status", QUEUE_SIZE)

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
        self.Ts = Ts
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)
        self.offboard_setpoint_counter_ = 0

        # robot state
        self.state = np.zeros(12)
        self.u_opt = [0.0,0.0,0.0,0.0]
        self.w_opt = [0.0,0.0,0.0,0.0]

        # tilt angle
        self.tilt_angle = varphi_0
        self.init_time = False
        self.dt = 0.0
        self.current_time = 0.0
        self.initial_time = 0.0

        # mpc flag
        self.mpc_flag = False
        self.counter = 0
        self.compute_mpc = True 
        self.mpc_status = 0
        self.comp_time = 0.0
        self.set_initial_reference = False
        self.p0 = np.array([0.0,0.0])

        # acados solver
        self.ocp = create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_ocp_" + self.ocp.model.name + ".json"
        )

        # solver initialization
        for stage in range(N_horizon + 1):
            self.acados_ocp_solver.set(stage, "x", X0)
            self.acados_ocp_solver.set(stage, "p", np.array([varphi_0]))

        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", U_ref)

    def timer_callback(self):
        # DO OFFBOARD STUFF ##
        self.counter += 1
        print(f"counter: {self.counter}")
        if (self.counter > 5*int(1/Ts) and self.mpc_status==0): 
            self.mpc_flag = True
            if not (self.init_time):
                self.initial_time = Clock().now().nanoseconds/1e9
                self.previous_time = self.initial_time
                self.dt = 0.0
                self.init_time = True
            else:
                self.current_time = Clock().now().nanoseconds/1e9 - self.initial_time
                self.dt = self.current_time - self.previous_time
                self.previous_time = self.current_time

        if not self.set_initial_reference:
                traj_descent[0,:] = self.state[0]*np.ones((1,N_traj))
                traj_descent[1,:] = self.state[1]*np.ones((1,N_traj))
                # self.p0 = np.array([X_ref[0],X_ref[1]])
                self.set_initial_reference = True

        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

            # Arm the vehicle
            self.arm()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

        if not self.mpc_flag:
            self.publish_offboard_control_mode_position()  
            self.publish_trajectory_setpoint(START_POSITION)
        else:
            # self.publish_offboard_control_mode_attitude()
            self.publish_offboard_control_mode_direct_actuator()   
            # self.publish_offboard_control_mode_body_rate() 

            if not (self.mpc_status == 0): 
                self.mpc_flag = False
                return

            # run controller
            # if (self.mpc_flag and self.compute_mpc): 
            self.mpc_update()
            # self.simple_controller_update()

            # update tilt angle
            tilt_vel = 1.0
            self.tilt_angle += self.dt * tilt_vel * v_max_absolute
            self.tilt_angle = np.clip(self.tilt_angle,0,np.deg2rad(50))
            self.publish_tilt_angle(self.tilt_angle)

            # publish control inputs
            # self.publish_vehicle_rates(self.w_opt)
            # self.publish_vehicle_attitude_setpoint(self.att_opt)
            self.publish_actuator_motors(self.u_opt,self.tilt_angle/(np.pi/2))

            # publish log values
            self.publish_log(self.mpc_status,self.w_opt,self.state,self.comp_time)

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

    def mpc_update(self):
        if (self.state[2] < -0.05):
            start_time = time.process_time()

            global X_ref

            # feed next z position into X_ref
            # X_ref[2] = X_ref[2] + self.Ts * descent_velocity
            # X_ref = spatial_tracking(self.state,traj_descent)
            # X_ref = spatial_tracking(self.state,traj_xy)

            # print(f"current time: {self.current_time}")
            X_ref = time_tracking(self.current_time,traj_descent_time)
            # print(X_ref)

            # set initial state constraint
            self.acados_ocp_solver.set(0, "lbx", self.state)
            self.acados_ocp_solver.set(0, "ubx", self.state)

            # update yref and parameters
            for j in range(N_horizon):
                yref = np.hstack((X_ref,U_ref))
                self.acados_ocp_solver.set(j, "yref", yref)
                self.acados_ocp_solver.set(j, "p", np.array([self.tilt_angle]))
            yref_N = X_ref
            self.acados_ocp_solver.set(N_horizon, "yref", yref_N)
            self.acados_ocp_solver.set(N_horizon, "p", np.array([self.tilt_angle]))

            # solve ocp
            self.mpc_status = self.acados_ocp_solver.solve()  
            print(f"mpc_status: {self.mpc_status}")

            # get first input
            self.u_opt = self.acados_ocp_solver.get(0, "u")      
            x_opt = self.acados_ocp_solver.get(1, "x")  

            # body rate control 
            c = (S_numeric(np.zeros(12),self.tilt_angle).T @ self.u_opt)[2]/T_max
            self.omega_d = [x_opt[9],x_opt[10],x_opt[11]]
            self.w_opt = self.omega_d + [c]

            self.att_d = [x_opt[5],x_opt[4],x_opt[3]]
            self.att_opt = self.att_d + [c]

            self.comp_time = time.process_time() - start_time
            print("* comp time = %5g seconds\n" % (self.comp_time))
        else:
            self.u_opt   = [0,0,0,0]
            self.w_opt   = [0,0,0,0]
            self.att_opt = [0,0,0,0]

    def simple_controller_update(self):
        start_time = time.process_time()
        phi = self.tilt_angle 
        x0,u0 = X_ref,U_ref/cos(phi)
        # phi = self.tilt_angle
        # x0,u0 = X_ref,U_ref/cos(phi)
        # Ac = A(x0,u0,phi)
        # Bc = B(x0,u0,phi)
        # K,_,_ = lqr(Ac,Bc,Q_mat,R_mat)
        self.u_opt = u0 - K @ (self.state - x0)
        X_ = self.state + T_horizon/N_horizon*f(self.state,self.u_opt,self.tilt_angle)
        self.w_opt = [X_[9][0],X_[10][0],X_[11][0],-np.sum(self.u_opt)/4]
        self.comp_time = time.process_time() - start_time
        print("* comp time = %5g seconds\n" % (self.comp_time))

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

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

    def publish_actuator_motors(self,u,normalized_tilt):
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]
        msg.control[4] = normalized_tilt
        msg.control[5] = normalized_tilt

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

    def publish_log(self,status,u,x,comptime):
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
        msg.varphi = self.tilt_angle

        msg.xref = X_ref.astype('float32')
        msg.uref = U_ref.astype('float32')

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

def main(args=None):
    rclpy.init(args=args)
    print("Starting model predictive controller...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # xd,yd,zd = 0.4,0.4,0.4

    # vbar = 0.5
    # p = np.array([self.state[0],self.state[1]])
    # d = self.p0 - p
    # R = np.linalg.norm(d)
    # v = vbar * (d/R)
    # pref = p + v*self.Ts
    # X_ref[0] = pref[0]
    # X_ref[1] = pref[1]
    # X_ref[6] = v[0]
    # X_ref[7] = v[1]
    # X_ref[8] = zd

    # X_ref[0] = X_ref[0] - np.sign(self.state[0])*self.Ts * xd
    # X_ref[1] = X_ref[1] - np.sign(self.state[1])*self.Ts * yd
    # X_ref[0] = 0.0
    # X_ref[1] = 0.0

    # deprecated

    
    # def simple_controller_update(self):
    #     zdot = self.state[8]
    #     zdot_d = 0.5
    #     k = 15.0
    #     T = (m*g + m*k*(zdot - zdot_d))/cos(self.tilt_angle)
    #     c = T/T_max

    #     # perform LQR for body rate stabilization
    #     # Q,R = np.diag([1,1,1]), np.diag([1,1,1,1])
    #     x0,u0 = self.state, np.array([c,c,c,c])
    #     # phi = self.tilt_angle
    #     # Ac = A(x0,u0,phi)[9:,9:]
    #     # Bc = B(x0,u0,phi)[9:,:]
    #     # K,_,_ = lqr(Ac,Bc,Q,R)
    #     # self.u_opt = u0
    #     self.w_opt = [0.0,0.0,0.0,-c]
    #     print(f"c: {c}")

    # self.omega   = np.array([self.state[9],self.state[10],self.state[11]])
    # chi   = F_numeric(self.state[5],self.state[4],self.state[3]) @ self.omega

    # # filter w_opt
    # taux,tauy,tauz = body_rate(self.w_opt,self.state)
    # print(f"tau: {taux}, {tauy}, {tauz}")
    # print(f"w_opt[3]: {self.w_opt[3]}")
    # self.u_filtered = np.linalg.pinv(S_numeric(np.zeros(12),self.tilt_angle).T) @ np.array([0,0,self.w_opt[3],taux,tauy,tauz])
    # self.publish_actuator_motors(self.u_filtered,self.tilt_angle/(np.pi/2))

    # self.u_filtered = self.u_opt + body_rate_lqr(omega,self.tilt_angle,omega_d)

    # if self.compute_mpc: self.compute_mpc = False
    # else: self.compute_mpc = True