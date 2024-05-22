from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg
from rosbags.typesys import Stores, get_typestore
from IPython import embed
import numpy as np 
import matplotlib.pyplot as plt 
from morphing_lander.morphing_lander_dynamics import f 
from morphing_lander.parameters import params_
from acados_template import AcadosSimSolver
import os 
from morphing_lander.morphing_lander_integrator import create_sim_solver_description

sim = create_sim_solver_description()
acados_sim_path = params_.get('acados_sim_path')
acados_integrator = AcadosSimSolver(
    sim,
    json_file=os.path.join(acados_sim_path, sim.model.name + '_acados_sim.json'),
    generate=True,
    build   =True
)

def ground_effect(z,u,phi):
    R = 0.23
    residual_acc = np.zeros(z.shape[0])
    residual_acc = (2.0-0.5*abs(z/R)-1.0)*params_.get('kT')*np.sum(u,axis=1)/params_.get('m')
    idx = np.where(np.abs(z)/R > 2)
    residual_acc[idx] = 0.0
    return residual_acc

def smooth(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ    
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop  ))

bagpath = Path('/home/m4pc/Desktop/rosbag2_successful_landing_cast')
topic_name = 'mpc_status'
type_path = '/home/m4pc/m4v2-code/m4_home/m4_ws/src/custom_msgs/msg/MPCStatus.msg'

###### Add custom type to typestore ######

# Read definitions to python strings.
msg_text = Path(type_path).read_text()

# Plain dictionary to hold message definitions.
add_types = {}

# Add definitions from one msg file to the dict.
add_types.update(get_types_from_msg(msg_text, 'custom_msgs/msg/MPCStatusOld'))
typestore = get_typestore(Stores.ROS2_FOXY)
typestore.register(add_types)

###### Post-Processing Step #######

# create reader instance and open for reading
N_samples = 0
x_vec = []
# x_next_vec = []
u_vec = []
phi_vec = []
t_vec = []
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == topic_name]  
    for connection, timestamp, rawdata in reader.messages(connections=connections):
         msg = reader.deserialize(rawdata, connection.msgtype)
         x_vec.append(np.array([msg.x,msg.y,msg.z,msg.thetaz,msg.thetay,msg.thetax,msg.dx,msg.dy,msg.dz,msg.omegax,msg.omegay,msg.omegaz]))
         #  x_next_vec.append(msg.xnext)
         u_vec.append(msg.input)
         phi_vec.append(msg.varphi)
         t_vec.append(msg.timestamp)
         N_samples += 1

x_vec = np.array(x_vec)
# x_next_vec = np.array(x_next_vec)
u_vec = np.array(u_vec)
phi_vec = np.array(phi_vec)
t_vec = np.array(t_vec)/1e6
dt_vec = np.diff(t_vec,axis=0)

SMOOTH = 21
for i in range(12):
    x_vec[:,i] = smooth(x_vec[:,i],SMOOTH)
for i in range(4):
    u_vec[:,i] = smooth(u_vec[:,i],SMOOTH)
phi_vec = smooth(phi_vec,SMOOTH)

x_dot_vec = np.diff(x_vec,axis=0)/np.expand_dims(dt_vec,axis=1)

f_vec = []
for i in range(x_vec.shape[0]-1):
    f_vec.append(f(x_vec[i,:],u_vec[i,:],phi_vec[i]))
f_vec = np.array(f_vec).squeeze()
fres = x_dot_vec - f_vec

x_next_vec = []
for i in range(x_vec.shape[0]-1):
    x_next_vec.append(acados_integrator.simulate(x=x_vec[i,:],u=u_vec[i,:],p=phi_vec[i]))
    # x_next_vec.append(x_vec[i]+dt_vec[i]*f_vec[i])
x_next_vec = np.array(x_next_vec).squeeze()

# ground_effect_acc = ground_effect(x_vec[:,2],u_vec,phi_vec)
# plt.plot(t_vec[1:],x_next_vec[:,8],'r')
# plt.plot(t_vec,x_vec[:,8],'b')

fig, axs = plt.subplots(3,3)
axs[0,0].plot(t_vec[1:],x_next_vec[:,0],'r')
axs[0,0].plot(t_vec,x_vec[:,0],'b')

axs[1,0].plot(t_vec[1:],x_next_vec[:,1],'r')
axs[1,0].plot(t_vec,x_vec[:,1],'b')

axs[2,0].plot(t_vec[1:],x_next_vec[:,2],'r')
axs[2,0].plot(t_vec,x_vec[:,2],'b')

axs[0,1].plot(t_vec[1:],x_next_vec[:,6],'r')
axs[0,1].plot(t_vec,x_vec[:,6],'b')

axs[1,1].plot(t_vec[1:],x_next_vec[:,7],'r')
axs[1,1].plot(t_vec,x_vec[:,7],'b')

axs[2,1].plot(t_vec[1:],x_next_vec[:,8],'r')
axs[2,1].plot(t_vec,x_vec[:,8],'b')

axs[0,2].plot(t_vec[1:],x_next_vec[:,5],'r')
axs[0,2].plot(t_vec,x_vec[:,5],'b')

axs[1,2].plot(t_vec[1:],x_next_vec[:,4],'r')
axs[1,2].plot(t_vec,x_vec[:,4],'b')

axs[2,2].plot(t_vec[1:],x_next_vec[:,3],'r')
axs[2,2].plot(t_vec,x_vec[:,3],'b')

fig, axs = plt.subplots(3,3)
axs[0,0].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,0],'r')

axs[1,0].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,1],'r')

axs[2,0].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,2],'r')

axs[0,1].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,6],'r')

axs[1,1].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,7],'r')

axs[2,1].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,8],'r')

axs[0,2].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,5],'r')

axs[1,2].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,4],'r')

axs[2,2].plot(t_vec[1:int(t_vec.size/2)],fres[:int(t_vec.size/2)-1,3],'r')

plt.show()
embed()
