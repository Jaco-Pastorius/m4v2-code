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

# Python imports
import numpy as np
import time 

# Morphing lander MPC import
from morphing_lander.morphing_lander_mpc import *

class OffboardControl(Node): 
    def __init__(self):
        super().__init__('OffboardControl')
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.actuator_motors_publisher_   = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", 10)
        self.trajectory_setpoint_publisher_   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        
        # subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
                                            VehicleOdometry,
                                            '/fmu/out/vehicle_odometry',
                                            self.vehicle_odometry_callback,
                                            qos_profile_sensor_data)
        self.vehicle_odometry_subscriber  # prevent unused variable warning
         
        # timer callback
        self.Ts = 0.01
        self.timer_ = self.create_timer(self.Ts, self.timer_callback)
        self.offboard_setpoint_counter_ = 0

        # robot state
        self.state = np.zeros(13)

        # tilt angle
        self.tilt_angle = 0.0
        self.init_time = False
        self.dt = 0.0
        self.previous_time = 0.0

        # mpc flag
        self.mpc_flag = False
        self.counter = 0

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
        self.counter+=1
        print(f"counter: {self.counter}")
        if (self.counter > 1000): self.mpc_flag = True
        
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

            # Arm the vehicle
            self.arm()

        # publish offboard mode heartbeat
        if not self.mpc_flag:
            self.publish_offboard_control_mode_position()  
            self.publish_trajectory_setpoint()
        else:
            self.publish_offboard_control_mode_direct_actuator()       

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def vehicle_odometry_callback(self, msg):
        # get state from odometry
        p = msg.position
        phi,th,psi = self.euler_from_quaternion([msg.q[1],msg.q[2],msg.q[3],msg.q[0]])
        v = msg.velocity
        o = msg.angular_velocity

        if not (self.init_time):
            self.previous_time = msg.timestamp
            self.dt = 0.0
            self.init_time = True
        else:
            self.dt = msg.timestamp - self.previous_time

        self.state = np.array([p[0],p[1],p[2],psi,th,phi,v[0],v[1],v[2],o[0],o[1],o[2],self.tilt_angle])
        self.q = np.array([p[0],p[1],p[2],psi,th,phi])
        self.u = np.array([v[0],v[1],v[2],o[0],o[1],o[2]])

        print(f"phi,th,psi: {phi:.2f},{th:.2f},{psi:.2f}".format())
        print(f"x,y,z: {p[0]:.2f},{p[1]:.2f},{p[2]:.2f}".format())

        # run mpc
        if (self.mpc_flag): self.mpc_update()

        # update time
        self.previous_time = msg.timestamp

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
        self.tilt_angle += self.dt/1.0e6 * u_opt[4] * v_max_absolute
        self.publish_actuator_motors(u_opt[:-1], np.clip(self.tilt_angle/(np.pi/2),0.0,1.0))

        print(f"dt: {self.dt/1.0e6}")

        print("* comp time = %5g seconds\n" % (time.process_time() - start_time))

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

    def publish_actuator_motors(self,u,desired_tilt_angle):
        msg = ActuatorMotors()
        msg.control[0] = u[0]
        msg.control[1] = u[1]
        msg.control[2] = u[2]
        msg.control[3] = u[3]
        msg.control[4] = desired_tilt_angle
        msg.control[5] = desired_tilt_angle

        print(f"desired tilt angle is: {desired_tilt_angle*np.pi/2}")

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
