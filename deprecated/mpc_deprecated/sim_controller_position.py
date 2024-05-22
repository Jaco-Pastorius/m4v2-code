import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import TiltAngle
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAttitudeSetpoint
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

import numpy as np 

# Python imports
import time
import numpy as np
from numpy import sin, cos,pi
import casadi as ca
from IPython import embed

class OptRes():
    def __init__(self,time,inputs,states,Xd,Ud):
        self.time = time
        self.inputs = inputs
        self.states = states
        self.Xd = Xd
        self.Ud = Ud

class positionMPC():
    def __init__(self, N):

        # plotting fontsize 
        self.fontsize = 10

        # Number of control intervals
        self.N = N

        # Setup problem
        self.opti = ca.Opti()

        # Parameters
        self.m = 3.5
        self.g = 9.81

        self.Tmax = 2*self.m*self.g
        self.Fmax = self.Tmax/2
        self.max_tilt = np.pi/4
        self.max_tilt_vel = np.pi/2

        # parameter current yaw
        self.psi = self.opti.parameter(1,1)

        # Create parameter which is current tilt angle
        self.varphi = self.opti.parameter(1,1)

        # Create parameter which is reference position
        self.X_ref = self.opti.parameter(6,1)

        # Create parameter which is the initial state
        self.X0 = self.opti.parameter(6,1)

        # Decision variables
        self.X = self.opti.variable(6, self.N)  # state trajectory (excluding self.X0)
        self.x = self.X[0, :]
        self.y = self.X[1, :]
        self.z = self.X[2, :]
        self.dx = self.X[3,:]
        self.dy = self.X[4,:]
        self.dz = self.X[5,:]
        # self.varphi = self.X[6,:]

        self.U = self.opti.variable(3, self.N)  # control trajectory (self.u1, self.u2) (no final control)
        self.phi = self.U[0, :]
        self.th = self.U[1, :]
        self.T =  self.U[2, :]
        # self.F =  self.U[3, :]
        # self.v = self.U[3,:]

        self.tf = self.opti.parameter()         # final time

        # Set the objective
        Q = ca.diag([10,1,1,1,1,1])
        cost_position = ca.sum1(ca.sum2((self.X-self.X_ref).T@Q@(self.X-self.X_ref)))
        cost_inputs   = 1e-5*ca.sum1(ca.sum2(self.U*self.U))
        final_cost = self.dz[-1]**2+ 1000*(self.varphi[-1]-ca.pi/2)**2
        alpha         = 1e-5
        beta          = 10

        self.opti.minimize(cost_position + alpha*cost_inputs)

        # Create gap closing constraints using RK4
        dt = self.tf / self.N  # length of a control interval
        x_next = self.rk4(self.X0,self.U[:,0],dt)
        self.opti.subject_to(self.X[:, 1] == x_next)
        for k in range(1,self.N):
            x_next = self.rk4(self.X[:, k-1],self.U[:, k],dt)
            self.opti.subject_to(self.X[:, k] == x_next)

        # path constraints
        self.opti.subject_to(self.T <= self.Tmax)
        self.opti.subject_to(0<=self.T)

        # self.opti.subject_to(self.varphi <= np.pi/2)
        # self.opti.subject_to(self.varphi >= 0.0)

        # self.opti.subject_to(self.v >= -self.max_tilt_vel)
        # self.opti.subject_to(self.v <= self.max_tilt_vel)

        self.opti.subject_to(self.phi**2 + self.th**2 < self.max_tilt**2)

        # Solve NLP using IPOPT
        p_opts = {'expand': True}
        p_opts = {'expand': True,'ipopt.print_level':0, 'print_time':0}
        s_opts = {'max_iter': 1e6}

        self.opti.solver('ipopt', p_opts, s_opts)


        # # Solve NLP using QRQP
        # opts = {
        #         # 'qpsol':'osqp',
        #         'qpsol':'qrqp',
        #         'print_iteration': False, 
        #         'print_header': False, 
        #         'print_status': False, 
        #         'qpsol_options.print_iter': False, 
        #         'qpsol_options.print_header': False, 
        #         'qpsol_options.error_on_fail': False, 
        #         'expand': True}
        
        # self.opti.solver('sqpmethod',opts)

    def solve(self):
        self.sol = self.opti.solve()
        return self.sol

    def eulzyx2rot(self, phi,th,psi):
        R = ca.MX.zeros(3,3)
        cx,cy,cz = cos(phi),cos(th),cos(psi)
        sx,sy,sz = sin(phi),sin(th),sin(psi)
        R[0,0] = cy*cz
        R[0,1] = cz*sx*sy-cx*sz
        R[0,2] = sx*sz+cx*cz*sy
        R[1,0] = cy*sz
        R[1,1] = cx*cz+sx*sy*sz
        R[1,2] = cx*sy*sz-cz*sx
        R[2,0] = -sy
        R[2,1] = cy*sx
        R[2,2] = cx*cy
        return R

    def f(self,in1, in2):
        x = in1[0]
        y = in1[1]
        z = in1[2]
        dx = in1[3]
        dy = in1[4]
        dz = in1[5]
        # varphi = in1[6]

        phi = in2[0]
        th = in2[1]
        T = in2[2]
        # v = in2[3]
        
        R = self.eulzyx2rot(phi,th,self.psi)

        # z is pointing down dynamics
        fB = ca.MX.zeros(3,1)
        fB[2] = -T*cos(self.varphi)

        pdd = 1/self.m * R@fB + ca.vertcat(0,0,self.g)

        f = ca.vertcat(dx,dy,dz,pdd[0],pdd[1],pdd[2])
        return f

    def rk4(self,x,u,dt):
        k1 = self.f(x,u)
        k2 = self.f(x + dt/2 * k1, u)
        k3 = self.f(x + dt/2 * k2, u)
        k4 = self.f(x + dt * k3, u)
        x_next = x + dt/6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

class positionOCP():
    def __init__(self,mpc,t_horizon):
        self.mpc = mpc
        self.t_horizon = t_horizon
        self.Xsol = None
        self.Usol = None

    def compute_trajectory(self,x,varphi,psi,xref,print_summary=False):
        self.mpc.opti.set_value(self.mpc.tf,self.t_horizon)
        self.mpc.opti.set_value(self.mpc.X0,ca.vertcat(x[0],x[1],x[2],x[3],x[4],x[5]))
        self.mpc.opti.set_value(self.mpc.X_ref,ca.vertcat(xref[0],xref[1],xref[2],xref[3],xref[4],xref[5]))
        self.mpc.opti.set_value(self.mpc.varphi,varphi)
        self.mpc.opti.set_value(self.mpc.psi,psi)

        # warm start
        if self.Xsol is not None: self.mpc.opti.set_initial(self.mpc.X, self.Xsol)
        if self.Usol is not None: self.mpc.opti.set_initial(self.mpc.U, self.Usol)
        sol = self.mpc.solve()
        self.Xsol =  sol.value(self.mpc.X)
        self.Usol =  sol.value(self.mpc.U)
        states = self.Xsol
        inputs = self.Usol
        time = np.linspace(0, self.t_horizon, self.mpc.N + 1)
        res = OptRes(time,inputs,states,None,None)

        return res

class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.thrust_publisher_   = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", 10)
        self.torque_publisher_   = self.create_publisher(VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint", 10)
        self.tilt_angle_publisher_ = self.create_publisher(TiltAngle, "/fmu/in/tilt_angle", 10)
        self.vehicle_attitude_setpoint_publisher_   = self.create_publisher(VehicleAttitudeSetpoint,"/fmu/in/vehicle_attitude_setpoint", 10)

        # subscribers
        self.vehicle_local_position_subscriber_ = self.create_subscription(
                                    VehicleLocalPosition,
                                    '/fmu/out/vehicle_local_position',
                                    self.vehicle_local_position_callback,
                                    qos_profile_sensor_data)
        self.vehicle_local_position_subscriber_  # prevent unused variable warning

        self.vehicle_attitude_subscriber_ = self.create_subscription(
                                            VehicleAttitude,
                                            '/fmu/out/vehicle_attitude',
                                            self.vehicle_attitude_callback,
                                            qos_profile_sensor_data)
        self.vehicle_attitude_subscriber_  # prevent unused variable warning

        self.offboard_setpoint_counter_ = 0
        self.Ts = 0.01 # 100 Hz
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Controller parameters
        self.m = 3.5  # kg
        self.g = 9.81  # m/s/s
        self.thrust_to_weight_ratio = 2.0 

        # Current altitude, descent speed, desired descent speed, and tilt_angle
        self.x,self.y,self.z = 0.0,0.0,0.0
        self.xdot,self.ydot,self.zdot = 0.0,0.0,0.0
        self.zdot_d = 0.1
        
        # Current attitude
        self.roll,self.pitch,self.yaw = 0.0,0.0,0.0

        # controller gains
        self.ei = 0.0
        self.ki = 1.0
        self.k = 100.0

        # tilt angle
        self.tilt_angle = 0.0
        self.tilt_vel = 20
        self.tilt_vel_max = 30

        # mpc parameters
        t_horizon, N = 2.0,20
        self.ocp = positionOCP(positionMPC(N), t_horizon)

        self.psi_not_set = True
        self.psi_0 = 0.0

    def vehicle_attitude_callback(self, msg):
        self.roll,self.pitch,self.yaw = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        print(f"attitude:({self.roll},{self.pitch},{self.yaw})")
        if self.psi_not_set:
            self.psi_0 = self.yaw
            self.psi_not_set = False

    def vehicle_local_position_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.xdot = msg.vx
        self.ydot = msg.vy
        self.zdot = msg.vz
        print(f"position:({self.x},{self.y},{self.z})")

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

            # Arm the vehicle
            self.arm()

        # Offboard_control_mode heartbeat
        self.publish_offboard_control_mode()

        # self.tilt_vel = np.clip(20,-self.tilt_vel_max,self.tilt_vel_max)

        # self.publish_tilt_angle()
        # self.publish_vehicle_thrust_and_torque_setpoint()
        # self.publish_trajectory_setpoint()
        # self.controller_update()

        self.mpc_update()
        # self.publish_vehicle_attitude_setpoint()

        # self.publish_actuator_servos()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def mpc_update(self):
        state = np.array([self.x,self.y,self.z,self.xdot,self.ydot,self.zdot])
        varphi = np.deg2rad(self.tilt_angle)
        xref = np.array([0,0,0,0,0,0])
        res = self.ocp.compute_trajectory(state,varphi,self.yaw,xref,print_summary=False)
        input_current = res.inputs[:, 0]
        thrust_body = [0.0,0.0,-self.thrust_to_throttle(input_current[2])]

        print(f"input_current: {input_current}")

        qd = self.quaternion_from_euler(input_current[0],input_current[1],self.yaw)
        self.publish_vehicle_attitude_setpoint(qd,thrust_body)

        # tilt_vel = input_current[3]
        # tilt_vel = np.deg2rad(20)
        # self.publish_tilt_angle(np.rad2deg(tilt_vel))

    def controller_update(self):
        print(f"zdot: {self.zdot}")
        print(f"zdot_d: {self.zdot_d}")
        e = - (self.zdot - self.zdot_d)
        self.ei += e*self.Ts
        self.u = self.m*self.g - self.k * e - self.ki * self.ei
        self.throttle = self.thrust_to_throttle(self.u)

        print(f"e:, throttle {e}, {self.throttle}")

        # self.tilt_vel = np.clip(20,-self.tilt_vel_max,self.tilt_vel_max)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False  # True for position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -2.5] 
        msg.yaw = 1.57 # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_thrust_and_torque_setpoint(self):
        msg_thrust = VehicleThrustSetpoint()
        msg_torque = VehicleTorqueSetpoint()

        msg_thrust.xyz[0] = 0.0
        msg_thrust.xyz[1] = 0.0
        msg_thrust.xyz[2] = -0.5

        msg_torque.xyz[0] = 0.0
        msg_torque.xyz[1] = 0.0
        msg_torque.xyz[2] = 0.0

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg_thrust.timestamp = timestamp
        msg_torque.timestamp = timestamp 
        self.thrust_publisher_.publish(msg_thrust)
        self.torque_publisher_.publish(msg_torque)

    def publish_vehicle_attitude_setpoint(self,q_desired,thrust_body):
        msg = VehicleAttitudeSetpoint()
        msg.q_d[0] = q_desired[0]
        msg.q_d[1] = q_desired[1]
        msg.q_d[2] = q_desired[2]
        msg.q_d[3] = q_desired[3]

        msg.thrust_body =  thrust_body
        print(f"thrust_body = ({thrust_body[0]},{thrust_body[1]},{thrust_body[2]})")
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_attitude_setpoint_publisher_.publish(msg)   

    def publish_tilt_angle(self,tilt_vel):
        self.tilt_angle += self.Ts * tilt_vel

        msg = TiltAngle()
        msg.value = self.tilt_angle
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.tilt_angle_publisher_.publish(msg)

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

    def euler_from_quaternion(self, quaternion):
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

    def throttle_to_thrust(self,throttle):
        return self.thrust_to_weight_ratio * self.m * self.g * throttle
        
    def thrust_to_throttle(self,thrust):
        return  np.clip(thrust / (self.thrust_to_weight_ratio * self.m * self.g),0.0,1.0)

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()