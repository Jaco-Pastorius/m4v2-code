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
from px4_msgs.msg import VehicleAngularVelocity

from px4_msgs.msg import VehicleOdometry

from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data

# Python imports
import numpy as np
from numpy import sin, cos,pi

# Acados
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import scipy.linalg
from casadi import SX, MX, DM, vertcat, sin, cos, cross
from numpy import array
from IPython import embed
import time 

# control allocation matrix
M = np.array([
    [-6.09,  4.64,  1.45, 29.0],
    [ 6.09, -4.64,  1.45, 29.0],
    [ 6.09,  4.64, -1.45, 29.0],
    [-6.09, -4.64, -1.45, 29.0]
])
M = M.transpose()

def eulzyx2rot(phi,th,psi):
    R = SX.zeros(3,3)
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

def getE_ZYX(phi,th,psi):
    cz,sz = cos(psi),sin(psi)
    cy,sy = cos(th) ,sin(th)
    out = SX.zeros(3,3)
    out[0,0] = 0
    out[0,1] = -sz
    out[0,2] = cy*cz
    out[1,0] = 0
    out[1,1] = cz
    out[1,2] = cy*cz
    out[2,0] = 1
    out[2,1] = 0
    out[2,2] = -sy
    return out

def getE_ZYX_dot(phi,th,psi,dphi,dth,dpsi):
    cz,sz = cos(psi),sin(psi)
    cy,sy = cos(th) ,sin(th)
    out = SX.zeros(3,3)
    out[0,0] = 0
    out[0,1] = -cz * dpsi
    out[0,2] = -sy*cz*dth -cy*sz*dpsi
    out[1,0] = 0
    out[1,1] = -sz*dpsi
    out[1,2] = -sy*cz*dth -cy*sz*dpsi
    out[2,0] = 0
    out[2,1] = 0
    out[2,2] = -cy*dth
    return out
    
def getEinvZYX(phi,th,psi):
    cz,sz = cos(psi),sin(psi)
    cy,sy = cos(th) ,sin(th)
    out = SX.zeros(3,3)
    out[0,0] = cz*sy/cy
    out[0,1] = sy*sz/cy
    out[0,2] = 1
    out[1,0] = -sz
    out[1,1] = cz
    out[1,2] = 0
    out[2,0] = cz/cy
    out[2,1] = sz/cy
    out[2,2] = 0
    return out
        
def skew(v):
    out = SX.zeros(3,3)
    out[0,0] = 0
    out[0,1] = -v[2]
    out[0,2] = v[1]
    out[1,0] = v[2]
    out[1,1] = 0
    out[1,2] = -v[0]
    out[2,0] = -v[1]
    out[2,1] = v[0]
    out[2,2] = 0
    return out

def export_robot_model() -> AcadosModel:
    model_name = "torque_dynamics"

    # parameters
    varphi = SX.sym("varphi",1)
    w = 0.0775
    d = 0.1325
    l = 0.16
    m = 3.283
    g = 9.81

    J = SX.zeros(3,3)
    J[0,0] = 0.002
    J[1,1] = 0.008
    J[2,2] = 0.007
    Jinv =  SX.zeros(3,3)
    Jinv[0,0] = 1.0/0.002
    Jinv[1,1] = 1.0/0.008
    Jinv[2,2] = 1.0/0.007
    
    # states
    x = SX.sym("x",8)
    z = x[0]
    dz = x[1]
    phi = x[2]
    th = x[3]
    psi = x[4]
    ox = x[5]
    oy = x[6]
    oz = x[7]

    # controls
    u = SX.sym("u",4)

    # xdot
    z_dot = SX.sym("z_dot")
    dz_dot = SX.sym("dz_dot")
    phi_dot = SX.sym("phi_dot")
    th_dot = SX.sym("th_dot")
    psi_dot = SX.sym("psi_dot")
    ox_dot = SX.sym("ox_dot")
    oy_dot = SX.sym("oy_dot")
    oz_dot = SX.sym("oz_dot")

    xdot = vertcat(z_dot,dz_dot,phi_dot,th_dot,psi_dot,ox_dot,oy_dot,oz_dot)

    # algebraic variables
    # z = None

    # parameters
    p = varphi

    # dynamics
    wrench = M @ u

    tau = vertcat(wrench[0],wrench[1],wrench[2])
    o   = vertcat(ox,oy,oz)
    R = eulzyx2rot(phi,th,psi)
    F = -tau[0]/(w+d*cos(varphi))
    T = wrench[3]

    fB = SX.zeros(3,1)   
    fB[1] = F*sin(varphi)
    fB[2] = -T*cos(varphi)

    # z is pointing down dynamics
    pdd = 1/m * R@fB + vertcat(0,0,g)
    chid = getEinvZYX(phi,th,psi) @ R @ o
    od = Jinv@(tau - cross(o,J@o))

    # finally write dynamics
    f_expl = vertcat(dz,pdd[2],chid[2],chid[1],chid[0],od[0],od[1],od[2])
    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model

X0 = np.array([0,0,0,0,0,0,0,0])  # Intitialize the states
N_horizon = 20  # Define the number of discretization steps
T_horizon = 1.0 # Define the prediction horizon
u_max = 1.0  # Define the max force allowed

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = np.diag([10,1,1,10,1,1,1,1])
    R_mat = 1e-5* np.diag([1,1,1,1])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+1,+1,+1,+1])
    ocp.constraints.idxbu = np.array([0,1,2,3])

    ocp.constraints.x0 = X0

    # set parameters
    ocp.parameter_values = np.zeros((1,))

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    # ocp.solver_options.nlp_solver_max_iter = 400
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp

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

        self.vehicle_angular_velocity_subscriber_ = self.create_subscription(
                                            VehicleAngularVelocity,
                                            '/fmu/out/vehicle_angular_velocity',
                                            self.vehicle_angular_velocity_callback,
                                            qos_profile_sensor_data)
        
        self.vehicle_attitude_subscriber_  # prevent unused variable warning

        self.offboard_setpoint_counter_ = 0
        self.Ts = 0.02
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)

        # Controller parameters
        self.m = 3.283   # kg
        self.g = 9.81  # m/s/s
        self.thrust_to_weight_ratio = 3.66

        self.w = 0.0775
        self.d = 0.1325
        self.l = 0.16

        # Current altitude, descent speed, desired descent speed, and tilt_angle
        self.x,self.y,self.z = 0.0,0.0,0.0
        self.dx,self.dy,self.dz = 0.0,0.0,0.0
        
        # Current attitude
        self.phi,self.th,self.psi = 0.0,0.0,0.0
        self.ox, self.oy, self.oz = 0.0,0.0,0.0

        # tilt angle
        self.tilt_angle = 0.0

        # create solver
        self.ocp = create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_ocp_" + self.ocp.model.name + ".json"
        )

        self.nx = self.ocp.model.x.size()[0]
        self.nu = self.ocp.model.u.size()[0]

        self.xcurrent = X0

        # initialize solver
        for stage in range(N_horizon + 1):
            self.acados_ocp_solver.set(stage, "x", 0.0 * np.ones(self.xcurrent.shape))
        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros((self.nu,)))

    def vehicle_attitude_callback(self, msg):
        self.phi,self.th,self.psi = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        print(f"attitude:({self.phi},{self.th},{self.psi})")

    def vehicle_angular_velocity_callback(self, msg):
        self.ox,self.oy,self.oz = msg.xyz[0],msg.xyz[1],msg.xyz[2]

    def vehicle_local_position_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.dx = msg.vx
        self.dy = msg.vy
        self.dz = msg.vz
        print(f"position:({self.x},{self.y},{self.z})")

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

            # Arm the vehicle
            self.arm()

        # Offboard_control_mode heartbeat
        self.publish_offboard_control_mode()

        # run mpc
        self.mpc_update()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def mpc_update(self):
        start_time = time.process_time()
        state = np.array([self.z,self.dz,self.phi,self.th,0.0,self.ox,self.oy,0.0])
        varphi = self.tilt_angle

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", state)
        self.acados_ocp_solver.set(0, "ubx", state)

        # update yref
        for j in range(N_horizon):
            yref = np.array([-1, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0])
            self.acados_ocp_solver.set(j, "yref", yref)  # set yref
            self.acados_ocp_solver.set(j, "p", varphi)   # set parameter
        yref_N = np.array([-1, 0, 0, 0, 0, 0, 0, 0])
        self.acados_ocp_solver.set(N_horizon, "yref", yref_N)

        # solve ocp
        status = self.acados_ocp_solver.solve()  

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")      
        # x_opt = self.acados_ocp_solver.get(0, "x")      

        print(f"u_opt={u_opt}")
        # print(f"x_opt={x_opt}")

        # u = np.array([[1,1,1,1],[-1,1,-1,1] ,[-1,-1,1,1],[1,-1,-1,1]]) @ u_opt
        # input_current = np.array([[0, self.w+ self.d*cos(varphi),0,0],[0,0, self.l*cos(varphi),0],[0,0,0, self.l*sin(varphi)],[1,0,0,0]]) @ u

        input_current = M@u_opt

        torque = [input_current[0]/12.18,input_current[1]/9.28,input_current[2]/2.9]
        thrust_body = [0.0,0,-input_current[3]/116.0]
        # torque = [0,0,0]
        # thrust_body = [0,0,-1]

        print(f"torque: {torque}")
        print(f"thrust: {thrust_body}")

        self.publish_vehicle_thrust_and_torque_setpoint(torque,thrust_body)
        print("* comp time = %5g seconds\n" % (time.process_time() - start_time))


    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False  # True for position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -2.5] 
        msg.yaw = 1.57 # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_thrust_and_torque_setpoint(self,torque,thrust):
        msg_thrust = VehicleThrustSetpoint()
        msg_torque = VehicleTorqueSetpoint()

        msg_thrust.xyz[0] = thrust[0]
        msg_thrust.xyz[1] = thrust[1]
        msg_thrust.xyz[2] = thrust[2]

        msg_torque.xyz[0] = torque[0]
        msg_torque.xyz[1] = torque[1]
        msg_torque.xyz[2] = torque[2]

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

    def publish_tilt_angle(self, tilt_vel):
        self.tilt_angle += self.Ts * tilt_vel

        msg = TiltAngle()
        msg.value = np.clip(self.tilt_angle,0.0,45)
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

        return roll, pitch, yaw    # def torque_normalized(self,torque):
    #     max_force_per_side = f_max*2
    #     max_torque = max_force_per_side * (self.w+self.d)
    #     return np.clip(torque/max_torque,-1.0,1.0)

    def quaternion_from_euler(self,phi,th,psi):
        w = cos(phi/2)*cos(th/2)*cos(psi/2) + sin(phi/2)*sin(th/2)*sin(psi/2)
        x = sin(phi/2)*cos(th/2)*cos(psi/2) - cos(phi/2)*sin(th/2)*sin(psi/2)
        y = cos(phi/2)*sin(th/2)*cos(psi/2) + sin(phi/2)*cos(th/2)*sin(psi/2)
        z = cos(phi/2)*cos(th/2)*sin(psi/2) - sin(phi/2)*sin(th/2)*cos(psi/2)
        return np.array([w,x,y,z])

    def throttle_to_thrust(self,throttle):
        return self.thrust_to_weight_ratio * self.m * self.g * throttle
        
    # def torque_normalized(self,torque):
    #     max_force_per_side = f_max*2
    #     max_torque = max_force_per_side * (self.w+self.d)
    #     return np.clip(torque/max_torque,-1.0,1.0)

    def thrust_to_throttle(self,thrust):
        # return  np.clip(thrust / (self.thrust_to_weight_ratio * self.m * self.g),0.0,1.0)
        return  np.clip(thrust/26.0,0.0,1.0)

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
