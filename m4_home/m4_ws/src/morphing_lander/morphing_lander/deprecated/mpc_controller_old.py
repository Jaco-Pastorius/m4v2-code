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
from px4_msgs.msg import TiltAngle
from px4_msgs.msg import InputRc
from px4_msgs.msg import TrajectorySetpoint
from custom_msgs.msg import TiltVel

# Python imports
import numpy as np
import time 

# Morphing lander MPC
from morphing_lander.morphing_lander_mpc import *

class OffboardControl(Node):
    def __init__(self):
        super().__init__('OffboardControl')
        
        # publishers
        self.vehicle_command_publisher_       = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.actuator_motors_publisher_       = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", 10)
        self.tilt_vel_publisher_              = self.create_publisher(TiltVel, "/tilt_vel", 10)
        self.trajectory_setpoint_publisher_   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        # subscriptions
        self.rc_input_subscriber_ = self.create_subscription(
                                    InputRc,
                                    '/fmu/out/input_rc',
                                    self.rc_listener_callback,
                                    qos_profile_sensor_data)
        self.rc_input_subscriber_         # rc input
        self.vehicle_odometry_subscriber = self.create_subscription(
                                            VehicleOdometry,
                                            '/fmu/out/vehicle_odometry',
                                            self.vehicle_odometry_callback,
                                            qos_profile_sensor_data)
        self.vehicle_odometry_subscriber  # vehicle odometry
        self.tilt_angle_subscriber_ = self.create_subscription(
                                    TiltAngle,
                                    '/fmu/in/tilt_angle',
                                    self.tilt_angle_callback,
                                    qos_profile_sensor_data)
        self.tilt_angle_subscriber_       # tilt angle

        # timer callback
        self.Ts = 0.01
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)
        self.offboard_setpoint_counter_ = 0

        # offboard mode
        self.offboard_flag = False
        self.offboard    = False 

        # mpc flag
        self.mpc_flag = False 

        # Configure RC inputs
        self.min = 1094 
        self.max = 1934 
        self.dead = 1514 

        # robot state
        self.state = np.zeros(13)

        # tilt angle
        self.tilt_angle = 0.0

        # acados solver
        self.ocp = create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            self.ocp, json_file="acados_ocp_" + self.ocp.model.name + ".json"
        )

        # solver initialization
        for stage in range(N_horizon + 1):
            self.acados_ocp_solver.set(stage, "x", X0)
        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", U_ref)

    def timer_callback(self):
        if (self.offboard_flag):
            # publish offboard mode heartbeat
            if not self.mpc_flag:
                self.publish_offboard_control_mode_position()
                self.publish_trajectory_setpoint()  
            else:
                self.publish_offboard_control_mode_direct_actuator()

            if not self.offboard:
                if (self.offboard_setpoint_counter_ == 10):
                    # go to offboard mode 
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

                    # Arm the vehicle
                    self.arm()

                    # vehicle is now in offboard mode and armed
                    self.offboard = True

                # stop the counter after reaching 11
                if (self.offboard_setpoint_counter_ < 11):
                    self.offboard_setpoint_counter_ += 1

        else:
            if self.offboard:
                self.disarm()  
                self.offboard = False 
                self.offboard_setpoint_counter_ = 0

    def rc_listener_callback(self, msg):
        # put vehicle in offboard mode and track fixed altitude
        if (msg.values[8] == self.max):
            self.offboard_flag = True
        else:
            self.offboard_flag = False

        # start model predictive controller for landing (same input that triggers tilt velocity automatic control)
        if (msg.values[7] == self.max):
            self.mpc_flag = True
        else:
            self.mpc_flag = False
        
    def tilt_angle_callback(self, msg):
        self.tilt_angle = np.deg2rad(msg.value)

    def vehicle_odometry_callback(self, msg):
        # get state from odometry
        p = msg.position
        phi,th,psi = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity

        self.state = np.array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2],self.tilt_angle])

        print(f"phi,th,psi: {phi:.2f},{th:.2f},{psi:.2f}".format())
        print(f"x,y,z: {p[0]:.2f},{p[1]:.2f},{p[2]:.2f}".format())

        # run mpc
        if (self.mpc_flag): self.mpc_update()
            
    def mpc_update(self):
        start_time = time.process_time()

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", self.state)
        self.acados_ocp_solver.set(0, "ubx", self.state)

        # update yref and parameters
        for j in range(N_horizon):
            yref = np.hstack((X_ref,U_ref))
            self.acados_ocp_solver.set(j, "yref", yref)
        yref_N = X_ref
        self.acados_ocp_solver.set(N_horizon, "yref", yref_N)

        # solve ocp
        status = self.acados_ocp_solver.solve()  

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")      
        print(f"u_opt: {u_opt}")

        # Publish actuator motors commanded by MPC controller
        self.publish_actuator_motors(u_opt[:-1])
        self.publish_tilt_vel(u_opt[4]) # this value is already normalized between -1 and 1 

        print("* comp time = %5g seconds\n" % (time.process_time() - start_time))

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    def publish_tilt_vel(self,tilt_vel):
        msg = TiltVel()
        msg.value = tilt_vel
        self.tilt_vel_publisher_.publish(msg)

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

    def publish_actuator_motors(self,u):
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]

        timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        msg.timestamp = timestamp
        self.actuator_motors_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = START_POSITION
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

def main(args=None):
    rclpy.init(args=args)
    print("Starting model predictive controller...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
