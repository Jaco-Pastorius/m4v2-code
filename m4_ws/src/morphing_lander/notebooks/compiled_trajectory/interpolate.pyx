import numpy as np
from scipy.interpolate import interp1d
cimport numpy as cnp

x_vec = np.load('x_vec.npy')
u_vec = np.load('u_vec.npy')
t_vec = np.load('t_vec.npy')
N_horizon = np.load('N_horizon.npy')
T_horizon = np.load('T_horizon.npy')

def create_interpolators():
    cdef int i
    cdef int x_cols = x_vec.shape[1]
    cdef int u_cols = u_vec.shape[1]
    
    x_interpolators = [interp1d(t_vec, x_vec[:, i], kind='linear', fill_value='extrapolate') for i in range(x_cols)]
    u_interpolators = [interp1d(t_vec, u_vec[:, i], kind='linear', fill_value='extrapolate') for i in range(u_cols)]
    
    return x_interpolators, u_interpolators

def interpolate_values(x_interpolators, u_interpolators, double t_new):
    cdef int i
    cdef int x_cols = len(x_interpolators)
    cdef int u_cols = len(u_interpolators)
    
    x_new = np.empty(x_cols, dtype=np.float64)
    u_new = np.empty(u_cols, dtype=np.float64)
    
    for i in range(x_cols):
        x_new[i] = x_interpolators[i](t_new)
    
    for i in range(u_cols):
        u_new[i] = u_interpolators[i](t_new)
    
    return x_new, u_new
