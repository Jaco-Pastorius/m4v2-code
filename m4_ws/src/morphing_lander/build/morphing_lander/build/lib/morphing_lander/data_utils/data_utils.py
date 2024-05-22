import torch
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg
from rosbags.typesys import Stores, get_typestore
import numpy as np
import matplotlib.pyplot as plt 
import plotly.graph_objects as go
from morphing_lander.mpc.dynamics import rk4
from casadi import external
rk4().generate('rk4.c')  # must be built by running: gcc -fPIC -shared rk4.c -o rk4.so
rk4_c = external("rk4", "/home/m4pc/m4v2-code/m4_ws/src/morphing_lander/morphing_lander/data_utils/rk4.so")

def get_data_from_rosbag(bag_path,type_path,topic_name):
    bagpath = Path(bag_path)

    # Add type to typestore
    msg_text = Path(type_path).read_text()
    add_types = {}
    add_types.update(get_types_from_msg(msg_text, 'custom_msgs/msg/MPCStatus'))
    typestore = get_typestore(Stores.ROS2_FOXY)
    typestore.register(add_types)

    # create reader instance and open for reading
    x_vec, u_vec, phi_vec, t_vec, x_ref_vec = [],[],[],[],[]
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]  
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            x_vec.append(msg.x)
            x_ref_vec.append(msg.xref)
            u_vec.append(msg.u)
            phi_vec.append(msg.varphi)
            t_vec.append(msg.timestamp)
    x_vec = np.array(x_vec)
    x_ref_vec = np.array(x_ref_vec)
    u_vec = np.array(u_vec)
    phi_vec = np.array(phi_vec)
    t_vec = np.array(t_vec)/1e6
    dt_vec = np.diff(t_vec,axis=0)

    # compute nominal model predictions
    x_next_vec = []
    for i in range(x_vec.shape[0]-1):
        x_next_vec.append(rk4_c(dt_vec[i],x_vec[i,:],u_vec[i,:],phi_vec[i]))
    x_next_vec = np.array(x_next_vec).squeeze()

    # compute the residuals 
    d     = x_next_vec - x_vec[1:]
    f_res = d/np.expand_dims(dt_vec,axis=1)

    data = {}
    data['d'] = d
    data['f_res'] = f_res
    data['x_vec'] = x_vec
    data['x_ref_vec'] = x_ref_vec
    data['x_next_vec'] = x_next_vec
    data['u_vec'] = u_vec
    data['phi_vec'] = phi_vec
    data['t_vec'] = t_vec - t_vec[0]
    data['dt_vec'] = dt_vec
    return data

def train_val_split(data,percent_val,state_idx,input_idx):
    N = data['x_vec'].shape[0]
    N_val = int(N*percent_val)
    target_tensor = torch.tensor(data['f_res'])
    # target_tensor = torch.tensor(data['d'])
    x_vec_tensor   = torch.tensor(data['x_vec'][:-1,state_idx])
    u_vec_tensor   = torch.tensor(data['u_vec'][:-1,input_idx])
    phi_vec_tensor = torch.tensor(data['phi_vec'])[:-1]
    train_data = torch.cat((target_tensor[:-N_val,:],x_vec_tensor[:-N_val,:],u_vec_tensor[:-N_val,:],phi_vec_tensor[:-N_val]),dim=1)
    val_data   = torch.cat((target_tensor[-N_val:,:],x_vec_tensor[-N_val:,:],u_vec_tensor[-N_val:,:],phi_vec_tensor[-N_val:]),dim=1)
    train_data = train_data.float()
    val_data   = val_data.float()
    return train_data, val_data

def plot_data(data):
    f_res = data['f_res']
    x_vec = data['x_vec']
    x_next_vec = data['x_next_vec']
    u_vec = data['u_vec']
    phi_vec = data['phi_vec']
    t_vec = data['t_vec']
    dt_vec = data['dt_vec']

    plt.figure()
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=t_vec,
            y=x_vec,
            name="Actual",
            line=dict(color="blue"),
        )
    )
    fig.show()
    # plt.plot(t_vec,x_vec[:,8],'b')
    # plt.plot(t_vec[1:],x_next_vec[:,8],'r')
    # plt.plot(t_vec[:-1],f_res[:,8],'r')

    # fig, axs = plt.subplots(3,3)
    # axs[0,0].plot(t_vec,x_vec[:,0],'b')
    # axs[1,0].plot(t_vec,x_vec[:,1],'b')
    # axs[2,0].plot(t_vec,x_vec[:,2],'b')
    # axs[0,1].plot(t_vec,x_vec[:,6],'b')
    # axs[1,1].plot(t_vec,x_vec[:,7],'b')
    # axs[2,1].plot(t_vec,x_vec[:,8],'b')
    # axs[0,2].plot(t_vec,x_vec[:,5],'b')
    # axs[1,2].plot(t_vec,x_vec[:,4],'b')
    # axs[2,2].plot(t_vec,x_vec[:,3],'b')

    # fig, axs = plt.subplots(3,3)
    # axs[0,0].plot(t_vec[:-1],f_res[:,0],'r')
    # axs[1,0].plot(t_vec[:-1],f_res[:,1],'r')
    # axs[2,0].plot(t_vec[:-1],f_res[:,2],'r')
    # axs[0,1].plot(t_vec[:-1],f_res[:,6],'r')
    # axs[1,1].plot(t_vec[:-1],f_res[:,7],'r')
    # axs[2,1].plot(t_vec[:-1],f_res[:,8],'r')
    # axs[0,2].plot(t_vec[:-1],f_res[:,5],'r')
    # axs[1,2].plot(t_vec[:-1],f_res[:,4],'r')
    # axs[2,2].plot(t_vec[:-1],f_res[:,3],'r')


def smooth(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ    
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop  ))