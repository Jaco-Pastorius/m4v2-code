import torch
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg
from rosbags.typesys import Stores, get_typestore
import numpy as np
import matplotlib.pyplot as plt 

# generate and build all compiled functions
import os
from casadi import external
from morphing_lander.mpc.dynamics import f, f_res, rk4

# change to compiled functions directory
os.chdir(os.path.dirname(os.path.abspath(__file__)) + '/compiled_functions')

# get c-compiled rk4 function
rk4().generate('rk4.c')
os.system('gcc -fPIC -shared rk4.c -o rk4.so')
rk4_c = external("rk4", "/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/data_utils/compiled_functions/rk4.so")

# get c-compiled f function
f().generate('f.c') 
os.system('gcc -fPIC -shared f.c -o f.so')
f_c = external("f", "/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/data_utils/compiled_functions/f.so")

# get c-compiled fres function
f_res().generate('f_res.c') 
os.system('gcc -fPIC -shared f_res.c -o f_res.so')
f_res_c = external("f_res", "/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/data_utils/compiled_functions/f_res.so")

# define labels
state_labels = [
    ('x (m)', r'$\dot{x}$ (m/s)', r'$\theta_x$ (rad)',r'$\omega_x$ (rad/s)' ),
    ('y (m)', r'$\dot{y}$ (m/s)', r'$\theta_y$ (rad)',r'$\omega_y$ (rad/s)' ),
    ('z (m)', r'$\dot{z}$ (m/s)', r'$\theta_z$ (rad)',r'$\omega_z$ (rad/s)' )
]

res_labels = [
    (r'$\dot{x}$ (m/s)', r'$\ddot{x}$ (m/s^2)', r'$\dot\theta_x$ (rad/s)',r'$\dot\omega_x$ (rad/s^2)' ),
    (r'$\dot{y}$ (m/s)', r'$\ddot{y}$ (m/s^2)', r'$\dot\theta_y$ (rad/s)',r'$\dot\omega_y$ (rad/s^2)' ),
    (r'$\dot{z}$ (m/s)', r'$\ddot{z}$ (m/s^2)', r'$\dot\theta_z$ (rad/s)',r'$\dot\omega_z$ (rad/s^2)' )
]

def get_data_from_rosbag(bag_path,type_path,topic_name,smooth_window=None):
    bagpath = Path(bag_path)

    # Add type to typestore
    msg_text = Path(type_path).read_text()
    add_types = {}
    add_types.update(get_types_from_msg(msg_text, 'custom_msgs/msg/MPCStatus'))
    typestore = get_typestore(Stores.ROS2_FOXY)
    typestore.register(add_types)

    # create reader instance and open for reading
    x_vec, u_vec, phi_vec, t_vec, x_ref_vec, f_vec = [],[],[],[],[],[]
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]  
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            x_vec.append(msg.x)
            x_ref_vec.append(msg.xref)
            u_vec.append(msg.u)
            phi_vec.append(msg.varphi)
            t_vec.append(msg.timestamp)
            f_vec.append(f_c(msg.x,msg.u,msg.varphi))
    x_vec = np.array(x_vec)
    x_ref_vec = np.array(x_ref_vec)
    u_vec = np.array(u_vec)
    phi_vec = np.array(phi_vec)
    t_vec = np.array(t_vec)/1e6
    dt_vec = np.diff(t_vec,axis=0)
    f_vec = np.array(f_vec).squeeze()

    # compute nominal model predictions
    x_next_vec, f_res_diff = [], []
    for i in range(x_vec.shape[0]-1):
        # x_next_vec.append(x_vec[i,:]+dt_vec[i]*f_vec[i,:])
        x_next_vec.append(rk4_c(dt_vec[i],x_vec[i,:],u_vec[i,:],phi_vec[i]))

    x_next_vec = np.array(x_next_vec).squeeze()

    # compute the residuals 
    d     = x_vec[1:] - x_next_vec
    f_res_int = d/dt_vec[:,np.newaxis]

    if smooth_window is not None:
        f_res_int_smooth = np.zeros(f_res_int.shape)
        for i in range(f_res_int.shape[1]):
            f_res_int_smooth[:,i] = smooth(f_res_int[:,i],smooth_window)
    else:
        f_res_int_smooth = f_res_int

    # get numerical state derivative (smooth x_vec first)
    x_vec_smooth = np.zeros(x_vec.shape)
    if smooth_window is not None:
        for i in range(x_vec.shape[1]):
            x_vec_smooth[:,i] = smooth(x_vec[:,i],smooth_window)
    else:
        x_vec_smooth = np.copy(x_vec)

    dx_vec = np.diff(x_vec_smooth,axis=0)/dt_vec[:,np.newaxis]
    f_res_diff = dx_vec - f_vec[:-1]

    data = {}
    data['d'] = d
    data['f_vec'] = f_vec
    data['f_res_int'] = f_res_int
    data['f_res_int_smooth'] = f_res_int_smooth
    data['f_res_diff'] = f_res_diff
    data['x_vec'] = x_vec
    data['x_ref_vec'] = x_ref_vec
    data['x_next_vec'] = x_next_vec
    data['u_vec'] = u_vec
    data['phi_vec'] = phi_vec
    data['t_vec'] = t_vec - t_vec[0]
    data['dt_vec'] = dt_vec
    return data

def get_data_from_rosbag_old(bag_path,type_path,topic_name,smooth_window=None):
    bagpath = Path(bag_path)

    # Add type to typestore
    msg_text = Path(type_path).read_text()
    add_types = {}
    add_types.update(get_types_from_msg(msg_text, 'custom_msgs/msg/MPCStatus'))
    typestore = get_typestore(Stores.ROS2_FOXY)
    typestore.register(add_types)

    # create reader instance and open for reading
    x_vec, u_vec, phi_vec, t_vec, x_ref_vec, f_vec = [],[],[],[],[],[]
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]  
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            x_ = [msg.x,msg.y,msg.z,msg.thetaz,msg.thetay,msg.thetax,msg.dx,msg.dy,msg.dz,msg.omegax,msg.omegay,msg.omegaz]
            x_vec.append(np.array(x_))
            x_ref_vec.append(msg.xref)
            u_vec.append(msg.input)
            phi_vec.append(msg.varphi)
            t_vec.append(msg.timestamp)
            f_vec.append(f_c(x_,msg.input,msg.varphi))
    x_vec = np.array(x_vec)
    x_ref_vec = np.array(x_ref_vec)
    u_vec = np.array(u_vec)
    phi_vec = np.array(phi_vec)
    t_vec = np.array(t_vec)/1e6
    dt_vec = np.diff(t_vec,axis=0)
    f_vec = np.array(f_vec).squeeze()

    # compute nominal model predictions
    x_next_vec, f_res_diff = [], []
    for i in range(x_vec.shape[0]-1):
        # x_next_vec.append(x_vec[i,:]+dt_vec[i]*f_vec[i,:])
        x_next_vec.append(rk4_c(dt_vec[i],x_vec[i,:],u_vec[i,:],phi_vec[i]))
        
    x_next_vec = np.array(x_next_vec).squeeze()

    # compute the residuals 
    d     = x_vec[1:] - x_next_vec
    f_res_int = d/dt_vec[:,np.newaxis]

    if smooth_window is not None:
        f_res_int_smooth = np.zeros(f_res_int.shape)
        for i in range(f_res_int.shape[1]):
            f_res_int_smooth[:,i] = smooth(f_res_int[:,i],smooth_window)
    else:
        f_res_int_smooth = f_res_int

    # get numerical state derivative (smooth x_vec first)
    x_vec_smooth = np.zeros(x_vec.shape)
    for i in range(x_vec.shape[1]):
        x_vec_smooth[:,i] = smooth(x_vec[:,i],21)
    
    dx_vec = np.diff(x_vec_smooth,axis=0)/dt_vec[:,np.newaxis]
    f_res_diff = dx_vec - f_vec[:-1]

    data = {}
    data['d'] = d
    data['f_vec'] = f_vec
    data['f_res_int'] = f_res_int
    data['f_res_int_smooth'] = f_res_int_smooth
    data['f_res_diff'] = f_res_diff
    data['x_vec'] = x_vec
    data['x_ref_vec'] = x_ref_vec
    data['x_next_vec'] = x_next_vec
    data['u_vec'] = u_vec
    data['phi_vec'] = phi_vec
    data['t_vec'] = t_vec - t_vec[0]
    data['dt_vec'] = dt_vec
    return data

def train_val_split(data,percent_val,state_idx,input_idx,include_phi,include_dt,output_idx):
    N = data['x_vec'].shape[0]
    N_val = int(N*percent_val)

    target_tensor  = torch.tensor(data['d'][:,output_idx])
    x_vec_tensor   = torch.tensor(data['x_vec'][:-1,state_idx])
    u_vec_tensor   = torch.tensor(data['u_vec'][:-1,input_idx])

    if include_phi:
        phi_vec_tensor = torch.tensor(data['phi_vec'])[:-1].unsqueeze(-1)
        train_data = torch.cat((target_tensor[:-N_val,:],x_vec_tensor[:-N_val,:],u_vec_tensor[:-N_val,:],phi_vec_tensor[:-N_val,:]),dim=1)
        val_data   = torch.cat((target_tensor[-N_val:,:],x_vec_tensor[-N_val:,:],u_vec_tensor[-N_val:,:],phi_vec_tensor[-N_val:,:]),dim=1)
    else:
        train_data = torch.cat((target_tensor[:-N_val,:],x_vec_tensor[:-N_val,:],u_vec_tensor[:-N_val,:]),dim=1)
        val_data   = torch.cat((target_tensor[-N_val:,:],x_vec_tensor[-N_val:,:],u_vec_tensor[-N_val:,:]),dim=1)    

    if include_dt:
        dt_tensor = torch.tensor(data['dt_vec']).unsqueeze(-1)
        train_data = torch.hstack((train_data,dt_tensor[:-N_val,:]))
        val_data = torch.hstack((val_data,dt_tensor[-N_val:,:]))

    return train_data.float(), val_data.float()  

def get_start_end_idx(t_vec,start_time,cutoff_time):
    if start_time is not None:
        start_idx = np.argmin(np.abs(t_vec - start_time))
    else:
        start_idx = 0
    if cutoff_time is not None:
        cutoff_idx = np.argmin(np.abs(t_vec - cutoff_time))
    else:
        cutoff_idx = t_vec.shape[0]
    return start_idx,cutoff_idx

def plot_12vector(t_vec,vec,handles=None,start_time=None, cutoff_time=None, name=None, title=None, labels=None, linestyle='b',display=True, overlay_vec=None,overlay_idx=None,overlay_linestyle='r'):

    start_idx,cutoff_idx = get_start_end_idx(t_vec,start_time,cutoff_time)

    if handles is None:
        plt.figure(figsize=(10, 8))
        fig, axs = plt.subplots(3, 4, figsize=(10, 8))
        handles = (fig,axs)

    idx_permutation = [0,1,2,8,7,6,3,4,5,9,10,11]

    # in column first row second format
    state_position = [(0,0),(1,0),(2,0),(2,2),(1,2),(0,2),(0,1),(1,1),(2,1),(0,3),(1,3),(2,3)]

    # Plot states
    handles[1][0, 0].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 0], linestyle)
    handles[1][1, 0].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 1], linestyle)
    handles[1][2, 0].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 2], linestyle)

    handles[1][0, 1].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 6], linestyle)
    handles[1][1, 1].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 7], linestyle)
    handles[1][2, 1].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 8], linestyle)

    handles[1][0, 2].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 5], linestyle)
    handles[1][1, 2].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 4], linestyle)
    handles[1][2, 2].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 3], linestyle)

    handles[1][0, 3].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 9], linestyle)
    handles[1][1, 3].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 10], linestyle)
    handles[1][2, 3].plot(t_vec[start_idx:cutoff_idx], vec[start_idx:cutoff_idx, 11], linestyle)

    if overlay_vec is not None:
        if len(overlay_vec.shape) == 1: overlay_vec = overlay_vec[:,np.newaxis]
        count = 0
        for idx in overlay_idx:
            handles[1][state_position[idx][0],state_position[idx][1]].plot(t_vec[start_idx:cutoff_idx], overlay_vec[start_idx:cutoff_idx, count], overlay_linestyle)
            count+=1

    if labels is not None:
        for i in range(3):
            for j in range(4):
                handles[1][i, j].set_xlabel('Time (sec)')
                handles[1][i, j].set_ylabel(labels[i][j])
                handles[1][i, j].grid(True)
    if title is not None:
        handles[0].suptitle(title)

    if display:
        plt.tight_layout()
        if name is not None:
            plt.savefig(name, format='png', dpi=300)  # Save the figure in high resolution PDF format
        plt.show()

    return handles[0], handles[1]

def plot_inputs(t_vec,u_vec,phi_vec,handles=None,start_time=None, cutoff_time=None, name=None,linestyle='b',display=True):
    input_vec = np.hstack((u_vec,phi_vec[:,np.newaxis]))

    start_idx,cutoff_idx = get_start_end_idx(t_vec,start_time,cutoff_time)

    if handles is None:
        plt.figure(figsize=(10, 8))
        fig, axs = plt.subplots(5, 1, figsize=(10, 8))
    else:
        fig, axs = handles[0],handles[1]

    # Plot inputs
    axs[0].plot(t_vec[start_idx:cutoff_idx], input_vec[start_idx:cutoff_idx, 0], linestyle)
    axs[1].plot(t_vec[start_idx:cutoff_idx], input_vec[start_idx:cutoff_idx, 1], linestyle)
    axs[2].plot(t_vec[start_idx:cutoff_idx], input_vec[start_idx:cutoff_idx, 2], linestyle)
    axs[3].plot(t_vec[start_idx:cutoff_idx], input_vec[start_idx:cutoff_idx, 3], linestyle)
    axs[4].plot(t_vec[start_idx:cutoff_idx], input_vec[start_idx:cutoff_idx, 4], linestyle)

    for i in range(4):
        axs[i].set_xlabel('Time (sec)')
        axs[i].set_ylabel(f"u{i+1}")
        axs[i].grid(True)
    axs[4].set_xlabel('Time (sec)')
    axs[4].set_ylabel(r"$\varphi$")
    axs[4].grid(True)
    plt.tight_layout()
    if name is not None:
        plt.savefig(name, format='png', dpi=300)  # Save the figure in high resolution PDF format
    plt.show()

    return fig, axs

def plot_data(data, start_time=None, cutoff_time=None, name=None, inputs=True, residuals=True):
    d     = data['d']
    x_vec = data['x_vec']
    x_ref_vec = data['x_ref_vec']
    x_next_vec = data['x_next_vec']
    u_vec = data['u_vec']
    phi_vec = data['phi_vec']
    t_vec = data['t_vec']
    f_res_int = data['f_res_int']
    f_res_int_smooth = data['f_res_int_smooth']
    f_res_diff = data['f_res_diff']
    f_vec = data['f_vec']
   
    # Plot states, reference, and model predictions
    fig,axs = plot_12vector(t_vec,
                            x_vec,
                            handles=None,
                            start_time=start_time,
                            cutoff_time=cutoff_time,
                            name=name,
                            title=None,
                            labels=None,
                            linestyle='b',
                            display=False)
    
    fig,axs = plot_12vector(t_vec,
                            x_ref_vec,
                            handles=(fig,axs),
                            start_time=start_time,
                            cutoff_time=cutoff_time,
                            name=name,
                            title=None,
                            labels=None,
                            linestyle='--k',
                            display=False)
    
    fig,axs = plot_12vector(t_vec[1:],
                            x_next_vec,
                            handles=(fig,axs),
                            start_time=start_time,
                            cutoff_time=cutoff_time,
                            name=name,
                            title=None,
                            labels=state_labels,
                            linestyle='r',
                            display=True)
    
    # plot residuals
    if residuals:
        fig,axs = plot_12vector(t_vec[1:],
                                d,
                                handles=None,
                                start_time=start_time,
                                cutoff_time=cutoff_time,
                                name=name,
                                title=None,
                                labels=res_labels,
                                linestyle='g',
                                display=True)

        # fig,axs = plot_12vector(t_vec[1:],
        #                         f_res_int_smooth,
        #                         handles=(fig,axs),
        #                         start_time=start_time,
        #                         cutoff_time=cutoff_time,
        #                         name=name,
        #                         title=None,
        #                         labels=res_labels,
        #                         linestyle='g',
        #                         display=True)

    # Plot inputs and tilt angle 
    if inputs:
        plot_inputs(t_vec,
                    u_vec,
                    phi_vec,
                    handles=None,
                    start_time=start_time,
                    cutoff_time=cutoff_time,
                    linestyle='b',
                    display=True)

def plot_z_data(data, start_time=None, cutoff_time=None):
    d = data['d']
    x_vec = data['x_vec']
    x_ref_vec = data['x_ref_vec']
    x_next_vec = data['x_next_vec']
    u_vec = data['u_vec']
    phi_vec = data['phi_vec']
    t_vec = data['t_vec']
    f_res_int = data['f_res_int']
    f_res_int_smooth = data['f_res_int_smooth']
    f_res_diff = data['f_res_diff']
    f_vec = data['f_vec']

    start_idx,cutoff_idx = get_start_end_idx(t_vec,start_time,cutoff_time)
   
    fig,axs = plt.subplots(3, 1, figsize=(10, 8))

    axs[0].plot(t_vec[start_idx:cutoff_idx],x_vec[start_idx:cutoff_idx,2],'b')
    axs[0].plot(t_vec[start_idx:cutoff_idx],x_next_vec[start_idx:cutoff_idx,2],'r')
    axs[0].set_xlabel('Time (sec)')
    axs[0].set_ylabel(r"$z$")
    axs[0].grid(True)

    axs[1].plot(t_vec[start_idx:cutoff_idx],x_vec[start_idx:cutoff_idx,8],'b')
    axs[1].plot(t_vec[start_idx:cutoff_idx],x_next_vec[start_idx:cutoff_idx,8],'r')
    axs[1].set_xlabel('Time (sec)')
    axs[1].set_ylabel(r"$\dot z$")
    axs[1].grid(True)

    # axs[2].plot(t_vec[start_idx:cutoff_idx],f_res_int[start_idx:cutoff_idx,8],'g')
    # axs[2].set_xlabel('Time (sec)')
    # axs[2].set_ylabel(r"$\ddot z$ (res)")
    # axs[2].grid(True)

    axs[2].plot(t_vec[start_idx:cutoff_idx],d[start_idx:cutoff_idx,8],'g')
    axs[2].set_xlabel('Time (sec)')
    axs[2].set_ylabel(r"$\ddot z$ (res)")
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

def smooth(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ    
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop  ))