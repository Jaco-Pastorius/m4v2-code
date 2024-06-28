# Abstract Base Class
from abc import ABC, abstractmethod

# Numpy imports
import numpy as np

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
from custom_msgs.msg import DriveVel

# Acados imports
from acados_template import AcadosOcpSolver

# Morphing lander imports
from morphing_lander.mpc.mpc                 import create_ocp_solver_description_with_int
from morphing_lander.mpc.trajectories        import traj_jump_time
from morphing_lander.mpc.parameters          import params_

from IPython import embed

# Get parameters
queue_size                 = params_.get('queue_size')
N_horizon                  = params_.get('N_horizon')
Ts                         = params_.get('Ts')
u_max                      = params_.get('u_max')
land_height                = params_.get('land_height')
takeoff_height             = params_.get('takeoff_height')
max_tilt_in_flight         = params_.get('max_tilt_in_flight')
max_tilt_on_land           = params_.get('max_tilt_on_land')
acados_ocp_path            = params_.get('acados_ocp_path')
acados_sim_path            = params_.get('acados_sim_path')
real_time_factor           = params_.get('real_time_factor')
use_residual_model         = params_.get('use_residual_model')
l4c_residual_model         = params_.get('l4c_residual_model')
generate_mpc               = params_.get('generate_mpc')
build_mpc                  = params_.get('build_mpc')
model_states_in_idx        = params_.get('model_states_in_idx')
model_inputs_in_idx        = params_.get('model_inputs_in_idx')
model_phi_in               = params_.get('model_phi_in')
model_dt_in                = params_.get('model_dt_in')
model_states_out_idx       = params_.get('model_states_out_idx')
model_ninputs              = params_.get('model_ninputs')
model_noutputs             = params_.get('model_noutputs')

# integral state gain
integral_gain              = params_.get('integral_gain')

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
        self.drive_vel_publisher_ = self.create_publisher(DriveVel, "/drive_vel", queue_size)

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

        # takeoff flag
        self.takeoff_flag = False
        self.mission_done = False

        # robot state
        self.state             = np.zeros(12)
        self.integral_state    = 0.0

        # tilt angle
        self.tilt_angle = 0.0

        # drive speed and turn speed
        self.drive_speed = 0.0
        self.turn_speed  = 0.0

        # time variables
        self.time_initialized = False
        self.initial_time = 0.0
        self.current_time = 0.0
        self.dt = 0.0

        # counter
        self.counter = 0

        # compile acados solver (ocp)
        # ocp  = create_ocp_solver_description()
        ocp = create_ocp_solver_description_with_int()
        self.acados_ocp_solver = AcadosOcpSolver(
            ocp, 
            json_file=os.path.join(acados_ocp_path, ocp.model.name + '_acados_ocp.json'),
            generate=generate_mpc,
            build   =build_mpc
        )

        # solver initialization
        for stage in range(N_horizon + 1):
            # self.acados_ocp_solver.set(stage, "x", np.zeros(12))
            self.acados_ocp_solver.set(stage, "x", np.zeros(13))
            if use_residual_model:
                l4c_params = l4c_residual_model.get_params(np.zeros((1,model_ninputs)))
                self.acados_ocp_solver.set(stage, "p", np.hstack((np.array([0.0]),l4c_params.squeeze())))
            else:
                self.acados_ocp_solver.set(stage, "p", np.array([0.0,0.0]))
                # self.acados_ocp_solver.set(stage, "p", np.array([0.0]))

        for stage in range(N_horizon):
            self.acados_ocp_solver.set(stage, "u", np.zeros(4))

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
            x_current   = np.copy(self.state) 
            phi_current = np.copy(self.tilt_angle) 

            # update reference (make sure reference starts from ground and ends on ground)
            x_ref,u_ref,tilt_vel,tracking_done = traj_jump_time(self.current_time)
            x_ref_copy = np.copy(x_ref)

            # # update reference based on joystick/RC commands
            # tracking_done = False
            # x_ref,u_ref,tilt_vel = self.get_reference()

            # update integral state
            self.integral_state += Ts * integral_gain * (x_current[2]-x_ref_copy[2])

            # check if robot has taken off
            self.takeoff_detector(x_current)

            # check if robot is grounded
            grounded_flag = self.ground_detector(x_current)

            # check if mission has finished
            self.mission_done_detector(grounded_flag)

            if self.mission_done:
                print("mission is done")
                # this means we are on the ground after having taken off
                # we need to switch off the thrusters and get to drive as fast as possible (if not there already)
                # also stop running the mpc
                u_opt = np.zeros(4,dtype='float')
                tilt_vel = u_max
                mpc_flag = False   
                # self.drive_speed = 1.0
                # self.turn_speed  = 0.0   
                # self.disarm()

            # run mpc
            x_next = np.zeros(12)
            if mpc_flag: 
                # u_opt,x_next,comp_time = self.mpc_update(x_current,phi_current,x_ref,u_ref)
                u_opt,x_next,comp_time = self.mpc_update_with_int(x_current,phi_current,x_ref_copy,u_ref,self.integral_state)
            else:
                u_opt = np.zeros(4,dtype='float')
                comp_time = -1.0

            # publish control inputs
            self.publish_actuator_motors(u_opt)

            # limit tilt velocity to ensure safety
            tilt_vel = self.limit_tilt_vel(tilt_vel,self.mission_done)

            # publish tilt velocity
            self.publish_tilt_vel(tilt_vel)

            print(f"tilt_angle : {self.tilt_angle}")
            print(f"tilt_vel : {tilt_vel}")

            # publish thrust for landing estimator
            self.publish_vehicle_thrust_setpoint(-np.sum(u_opt)/4)

            # publish log values
            self.publish_log(x_current,u_opt,x_ref,u_ref,self.tilt_angle,tilt_vel,self.mpc_status,comp_time,tracking_done,grounded_flag,x_next)

    # timer methods
    def mpc_update(self,xcurrent,phicurrent,x_ref,u_ref):
        start_time = time.process_time()

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", xcurrent)
        self.acados_ocp_solver.set(0, "ubx", xcurrent)
        
        # solve ocp
        self.mpc_status = self.acados_ocp_solver.solve()  
        print(f"mpc_status: {self.mpc_status}")

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")   

        # get predicted next state 
        x_next = self.acados_ocp_solver.get(1, "x") 

        # set the reference and parameters
        if use_residual_model:
            cond_vec = []
            print("using residual model")
            for j in range(N_horizon):
                yref = np.hstack((x_ref,u_ref))
                self.acados_ocp_solver.set(j, "yref", yref)
                x_ = self.acados_ocp_solver.get(j,"x")
                u_ = self.acados_ocp_solver.get(j,"u")
                cond = np.hstack((x_[model_states_in_idx],u_[model_inputs_in_idx]))
                if model_phi_in:
                    cond = np.hstack((cond,phicurrent))
                if model_dt_in:
                    cond = np.hstack((cond,Ts))
                cond_vec.append(cond)
            params = l4c_residual_model.get_params(np.stack(cond_vec,axis=0))
            for jj in range(N_horizon):
                self.acados_ocp_solver.set(jj, "p", np.hstack((np.array([phicurrent]),params[jj])))
        else:
            for j in range(N_horizon):
                yref = np.hstack((x_ref,u_ref))
                self.acados_ocp_solver.set(j, "yref", yref)
                self.acados_ocp_solver.set(j, "p", np.array([phicurrent]))

        comp_time = time.process_time() - start_time
        print("* comp time = %5g seconds\n" % (comp_time))

        return u_opt, x_next, comp_time
    
    def mpc_update_with_int(self,xcurrent,phicurrent,x_ref,u_ref,integral_state):
        start_time = time.process_time()

        # pad with integral state
        xcurrent = np.hstack((xcurrent,integral_state))
        x_ref    = np.hstack((x_ref,0.0))

        # set initial state constraint
        self.acados_ocp_solver.set(0, "lbx", xcurrent)
        self.acados_ocp_solver.set(0, "ubx", xcurrent)
        
        # solve ocp
        self.mpc_status = self.acados_ocp_solver.solve()  
        print(f"mpc_status: {self.mpc_status}")

        # get first input
        u_opt = self.acados_ocp_solver.get(0, "u")   

        # get predicted next state 
        x_next = self.acados_ocp_solver.get(1, "x") 

        # set the reference and parameters
        for j in range(N_horizon):
            yref = np.hstack((x_ref,u_ref))
            self.acados_ocp_solver.set(j, "yref", yref)
            self.acados_ocp_solver.set(j, "p", np.array([phicurrent,x_ref[2]]))

        comp_time = time.process_time() - start_time
        print("* comp time = %5g seconds\n" % (comp_time))

        return u_opt, x_next[:-1], comp_time

    def ground_detector(self,x_current):
        grounded_flag = False
        if np.absolute(x_current[2])<np.absolute(land_height):
            grounded_flag = True
        return grounded_flag

    def takeoff_detector(self,x_current):
        if np.absolute(x_current[2])>np.absolute(takeoff_height):
            self.takeoff_flag = True

    def mission_done_detector(self,grounded_flag):
        if self.takeoff_flag and grounded_flag:
            self.mission_done = True

    def initialize_time(self):
        self.initial_time = Clock().now().nanoseconds/1e9
        self.previous_time = self.initial_time
        self.dt = 0.0
        self.time_initialized = True

    def advance_time(self):
        if self.time_initialized:
            self.current_time = real_time_factor * (Clock().now().nanoseconds/1e9 - self.initial_time)
            self.dt = self.current_time - self.previous_time
            self.previous_time = self.current_time

    def limit_tilt_vel(self,tilt_vel,mission_done_flag):
        if not mission_done_flag: 
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
    
    @abstractmethod
    def get_reference(self):
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

    def publish_drive_vel(self,drive_speed,turn_speed):
        msg = DriveVel()
        msg.drivespeed = drive_speed
        msg.turnspeed  = turn_speed
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.drive_vel_publisher_.publish(msg)

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

    def publish_log(self,x,u,x_ref,u_ref,tilt_angle,tilt_vel,status,comptime,tracking_done,grounded_flag,x_next):
        msg = MPCStatus()

        msg.x = x.astype('float32')
        msg.xref = x_ref.astype('float32')
        msg.xnext = x_next.astype('float32')

        msg.u = u.astype('float32')
        msg.uref = u_ref.astype('float32')

        msg.varphi  = np.rad2deg(tilt_angle)
        msg.tiltvel = np.rad2deg(tilt_vel)

        msg.status = status
        msg.comptime = comptime
        msg.trackingdone = tracking_done
        msg.grounded    = grounded_flag
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds

        self.mpc_status_publisher_.publish(msg)

    @abstractmethod
    def publish_actuator_motors(self):
        pass



    # update yref and parameters
    # for j in range(N_horizon):
    #     yref = np.hstack((x_ref,u_ref))
    #     self.acados_ocp_solver.set(j, "yref", yref)
        # if use_residual_model:
        #     l4c_params = l4c_residual_model.get_params(expand_dims(xcurrent,axis=0))
        #     self.acados_ocp_solver.set(j, "p", np.hstack((phicurrent,l4c_params.squeeze())))
        # else:
        #     self.acados_ocp_solver.set(j, "p", np.array([phicurrent]))
    # yref_N = x_ref
    # self.acados_ocp_solver.set(N_horizon, "yref", yref_N)
    # if use_residual_model:
    #     l4c_params = l4c_residual_model.get_params(expand_dims(xcurrent,axis=0))
    #     self.acados_ocp_solver.set(N_horizon, "p", np.hstack((phicurrent,l4c_params.squeeze())))
    # else:
    #     self.acados_ocp_solver.set(N_horizon, "p", np.array([phicurrent]))

    # # emergency descent
    # self.set_emergency_xyz_trajectory = False 
    # self.emergency_xyz_position = np.zeros(3,dtype='float')
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
    #         self.emergency_t0                 = np.copy(self.current_time)
    #         self.emergency_xyz_position       = np.copy(self.state[:3])
    #         self.set_emergency_xyz_trajectory = True
    #     x_ref,u_ref = emergency_descent_time(self.current_time - self.emergency_t0,self.emergency_xyz_position)
    #     tilt_vel = -u_max  # go to drone configuration for more controllability

    #     # publish trajectory setpoint for land detector
    #     msg = TrajectorySetpoint()
    #     msg.velocity = [0.0,0.0,x_ref[8]]
    #     self.trajectory_setpoint_publisher_.publish(msg)