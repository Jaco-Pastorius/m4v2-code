# Abstract Base Class
from abc import ABC, abstractmethod

# Numpy imports
from numpy import zeros,array,copy,hstack,sum,absolute,rad2deg

# Python imports
import os 
import time

# ROS imports
from rclpy.node  import Node
from rclpy.clock import Clock
from rclpy.qos   import qos_profile_sensor_data

# Message imports
from px4_msgs.msg    import VehicleCommand
from px4_msgs.msg    import OffboardControlMode
from px4_msgs.msg    import ActuatorMotors
from px4_msgs.msg    import TrajectorySetpoint
from px4_msgs.msg    import TiltAngle
from px4_msgs.msg    import VehicleThrustSetpoint
from custom_msgs.msg import MPCStatus
from custom_msgs.msg import TiltVel

# Acados imports
from acados_template import AcadosOcpSolver

# Morphing lander imports
from morphing_lander.morphing_lander_mpc import create_ocp_solver_description
from morphing_lander.trajectories        import traj_jump_time, emergency_descent_time
from morphing_lander.parameters          import params_

# Get parameters
queue_size                 = params_.get('queue_size')
N_horizon                  = params_.get('N_horizon')
Ts                         = params_.get('Ts')
u_max                      = params_.get('u_max')
land_height                = params_.get('land_height')
max_tilt_in_flight         = params_.get('max_tilt_in_flight')
max_tilt_on_land           = params_.get('max_tilt_on_land')

class MPCBase(Node,ABC): 
    def __init__(self):
        super().__init__('MPCBase')

        # publishers 
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", queue_size)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", queue_size)
        self.actuator_motors_publisher_   = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", queue_size)
        self.trajectory_setpoint_publisher_   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", queue_size)
        self.tilt_angle_publisher_   = self.create_publisher(TiltAngle, "fmu/in/tilt_angle", queue_size)
        self.mpc_status_publisher_   = self.create_publisher(MPCStatus, "/mpc_status", queue_size)
        self.vehicle_thrust_setpoint_publisher_   = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", queue_size)
        self.tilt_vel_publisher_ = self.create_publisher(TiltVel, "/tilt_vel", queue_size)
        
        # subscribers
        self.tilt_angle_subscriber_ = self.create_subscription(
                                    TiltAngle,
                                    '/fmu/in/tilt_angle',
                                    self.tilt_angle_callback,
                                    qos_profile_sensor_data)
        self.tilt_angle_subscriber_

        # timer callback
        self.timer_ = self.create_timer(Ts, self.timer_callback)

        # offboard mode
        self.offboard_setpoint_counter_ = 0
        self.offboard = False

        # mpc flag
        self.mpc_status = 0

        # robot state
        self.state = zeros(12)

        # tilt angle
        self.tilt_angle = 0.0

        # time variables
        self.time_initialized = False
        self.initial_time = 0.0
        self.current_time = 0.0
        self.dt = 0.0

        # counter
        self.counter = 0

        # change to current directory
        acados_models_path = os.getenv("HOME") +'/m4v2-code/m4_home/m4_ws/src/morphing_lander/morphing_lander/acados_models/'

        # compile acados solver
        ocp = create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            ocp, 
            json_file=os.path.join(acados_models_path, ocp.model.name + '_acados_ocp.json'),
            generate=False,
            build   =False
        )

        # solver initialization
        for stage in range(N_horizon + 1):
            self.acados_ocp_solver.set(stage, "x", zeros(12))
            self.acados_ocp_solver.set(stage, "p", array([0.0]))

        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", zeros(4))

    # timer callback
    def timer_callback(self):
        # check if failsafe should be activated
        failsafe_flag = self.failsafe_trigger()

        # if failsafe, timer should do nothing, manual control should be regained
        # tilt velocity should be -1.0 so that it goes to drone configuration
        if failsafe_flag:
            print("failsafe activated !")
            self.publish_tilt_vel(-1.0)
            return

        # get offboard flag
        offboard_flag = self.offboard_mode_trigger()
        
        # switch to offboard mode
        if (not self.offboard) and offboard_flag: 
            self.switch_to_offboard() # sets self.offboard flag
        
        # if statement guarantees that offboard mode is exited when offboard switch is pushed other way
        if self.offboard and offboard_flag:
            # keep offboard mode alive by publishing heartbeat
            self.publish_offboard_control_mode_direct_actuator() 

            # trigger mpc
            mpc_flag = self.mpc_trigger()

            # when mpc is triggered initialize the time
            if mpc_flag and (not self.time_initialized): self.initialize_time()
            
            # as long as node is alive continue advancing time
            if self.time_initialized: self.advance_time()

            # get current state and tilt angle (self.state is updated by odometry callback)
            x_current   = copy(self.state) 
            phi_current = copy(self.tilt_angle) 

            # update reference (make sure reference starts from ground and ends on ground)
            x_ref,u_ref,tilt_vel,tracking_done = traj_jump_time(self.current_time)

            # check if robot is grounded
            grounded_flag = self.ground_detector(x_current)

            if tracking_done and grounded_flag:
                print("tracking done and grounded")
                # this means we are on the ground and have already executed the trajectory
                # we need to switch off the thrusters and get to drive as fast as possible (if not there already)
                # also stop running the mpc
                u_opt = zeros(4,dtype='float')
                tilt_vel = u_max
                mpc_flag = False      
                self.disarm()

            # run mpc
            if mpc_flag: 
                u_opt,comp_time = self.mpc_update(x_current,phi_current,x_ref,u_ref)
            else:
                u_opt = zeros(4,dtype='float')
                comp_time = -1.0

            # publish control inputs
            self.publish_actuator_motors(u_opt)

            # limit tilt velocity to ensure safety
            tilt_vel = self.limit_tilt_vel(tilt_vel,grounded_flag)

            # publish tilt velocity
            self.publish_tilt_vel(tilt_vel)

            print(f"tilt_angle : {self.tilt_angle}")
            print(f"tilt_vel : {tilt_vel}")

            # publish thrust for landing estimator
            self.publish_vehicle_thrust_setpoint(-sum(u_opt)/4)

            # publish log values
            self.publish_log(x_current,u_opt,x_ref,u_ref,self.tilt_angle,tilt_vel,self.mpc_status,comp_time)

    # timer methods
    def mpc_update(self,xcurrent,phicurrent,x_ref,u_ref):
        start_time = time.process_time()

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", xcurrent)
        self.acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref and parameters
        for j in range(N_horizon):
            yref = hstack((x_ref,u_ref))
            self.acados_ocp_solver.set(j, "yref", yref)
            self.acados_ocp_solver.set(j, "p", array([phicurrent]))
        yref_N = x_ref
        self.acados_ocp_solver.set(N_horizon, "yref", yref_N)
        self.acados_ocp_solver.set(N_horizon, "p", array([phicurrent]))

        # solve ocp
        self.mpc_status = self.acados_ocp_solver.solve()  
        print(f"mpc_status: {self.mpc_status}")

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")      

        comp_time = time.process_time() - start_time
        print("* comp time = %5g seconds\n" % (comp_time))

        return u_opt, comp_time
    
    def ground_detector(self,x_current):
        grounded_flag = False
        if absolute(x_current[2])<absolute(land_height):
            grounded_flag = True
        return grounded_flag

    def initialize_time(self):
        self.initial_time = Clock().now().nanoseconds/1e9
        self.previous_time = self.initial_time
        self.dt = 0.0
        self.time_initialized = True

    def advance_time(self):
        if self.time_initialized:
            self.current_time = Clock().now().nanoseconds/1e9 - self.initial_time
            self.dt = self.current_time - self.previous_time
            self.previous_time = self.current_time

    def limit_tilt_vel(self,tilt_vel,grounded_flag):
        if not grounded_flag: 
            if (self.tilt_angle >= max_tilt_in_flight and tilt_vel > 0.0):
                tilt_vel = 0.0
        else:
            if (self.tilt_angle >= max_tilt_on_land and tilt_vel > 0.0):
                tilt_vel = 0.0
        return tilt_vel

    def switch_to_offboard(self):
        self.publish_offboard_control_mode_direct_actuator() 
        # go to offboard mode after 10 setpoints and stop counter after reaching 11 
        if (self.offboard_setpoint_counter_ == 10):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
            self.offboard = True
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def failsafe_trigger(self):
        failsafe_flag = False
        # if mpc fails then always go to failsafe
        if self.mpc_status != 0:
            failsafe_flag = True
        return failsafe_flag

    @abstractmethod
    def mpc_trigger(self):
        pass
    
    @abstractmethod
    def offboard_mode_trigger(self):
        pass
    
    # subscription callbacks
    def tilt_angle_callback(self, msg):
        self.tilt_angle = msg.value

    # publisher methods
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

    def publish_vehicle_thrust_setpoint(self,c):
        msg = VehicleThrustSetpoint()
        msg.xyz = [0.0,0.0,c]
        self.vehicle_thrust_setpoint_publisher_ .publish(msg)

    def publish_tilt_angle(self,tilt_angle):
        msg = TiltAngle()
        msg.value = tilt_angle
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.tilt_angle_publisher_.publish(msg)

    def publish_tilt_vel(self,tilt_vel):
        msg = TiltVel()
        msg.value = tilt_vel
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.tilt_vel_publisher_.publish(msg)

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

        msg.varphi  = rad2deg(tilt_angle)
        msg.tiltvel = rad2deg(tilt_vel)

        msg.xref = x_ref.astype('float32')
        msg.uref = u_ref.astype('float32')

        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.mpc_status_publisher_.publish(msg)

    @abstractmethod
    def publish_actuator_motors(self):
        pass


    # # emergency descent
    # self.set_emergency_xyz_trajectory = False 
    # self.emergency_xyz_position = zeros(3,dtype='float')
    # self.emergency_t0 = 0.0

    # emergency_descent_velocity = params_.get('emergency_descent_velocity')

    # if tracking_done and (not grounded_flag):
    #     print("tracking done but not grounded")
    #     # this means we are in the air even after the trajectory has ended
    #     # this could happen because the trajectory finishes in the air 
    #     # it could also happen if the tracking has gone wrong
    #     # in both cases we need to land the vehicle
    #     # strategy: override reference with a trajectory that tracks to the ground at the current (x,y) position
    #     # also set u_ref to gravity compensation and tilt_vel to -1.0 (i.e. go as close as possible to drone configuration)
    #     if not self.set_emergency_xyz_trajectory:
    #         self.emergency_t0                 = copy(self.current_time)
    #         self.emergency_xyz_position       = copy(self.state[:3])
    #         self.set_emergency_xyz_trajectory = True
    #     x_ref,u_ref = emergency_descent_time(self.current_time - self.emergency_t0,self.emergency_xyz_position)
    #     tilt_vel = -u_max  # go to drone configuration for more controllability

    #     # publish trajectory setpoint for land detector
    #     msg = TrajectorySetpoint()
    #     msg.velocity = [0.0,0.0,x_ref[8]]
    #     self.trajectory_setpoint_publisher_.publish(msg)