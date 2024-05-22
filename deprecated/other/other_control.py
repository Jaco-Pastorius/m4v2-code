
   
# def simple_controller_update(self,x_ref,u_ref,tilt_angle):
#     start_time = time.process_time()
#     phi = tilt_angle 
#     x0,u0 = x_ref,u_ref/cos(phi)
#     # phi = self.tilt_angle
#     # x0,u0 = X_ref,U_ref/cos(phi)
#     # Ac = A(x0,u0,phi)
#     # Bc = B(x0,u0,phi)
#     # K,_,_ = lqr(Ac,Bc,Q_mat,R_mat)
#     self.u_opt = u0 - K @ (self.state - x0)
#     X_ = self.state + T_horizon/N_horizon*f(self.state,self.u_opt,self.tilt_angle)
#     self.w_opt = [X_[9][0],X_[10][0],X_[11][0],-np.sum(self.u_opt)/4]
#     self.comp_time = time.process_time() - start_time
#     print("* comp time = %5g seconds\n" % (self.comp_time))

# Q,R = np.diag([1,1,1,1,1,1,1,1,1,1,1,1]), np.diag([1,1,1,1])
# Nphi = 100
# K_array = np.zeros((4,12,Nphi))
# phi_ = np.linspace(0,np.pi/2-0.1,Nphi)
# for i in range(Nphi):
#     phi = phi_[i]
#     print(phi)
#     Ac = A(X_ref,U_ref/cos(phi),phi)
#     Bc = B(X_ref,U_ref/cos(phi),phi)
#     K,_,_ = lqr(Ac,Bc,Q_mat,R_mat)
#     K_array[:,:,i] = K
# embed()
# K,_,_ = lqr(Ac,Bc,Q,R)


## DEPRECATED

# x0_dummy = np.zeros(9)

# x0 = np.hstack((x0_dummy, np.array([0,0,0])))
# u0 = np.array([0,0,0,0])
# phi = 0.0
# Q = np.diag([1,1,1,1,1,1,1,1,1,1,1,1])
# R = np.diag([1,1,1,1])
# rk = np.linalg.matrix_rank(ctrb(A(x0,u0,phi),B(x0,u0,phi)))
# Ac = A(x0,u0,phi)[9:.9:]
# Bc = B(x0,u0,phi)[9:,:]
# K,_,_ = lqr(Ac,Bc,Q,R)
# print(K)
# print(rk)
# embed()

# def body_rate(r,y):
#     # declare global variables
#     global ex_prev,ey_prev,ez_prev,ex_i,ey_i,ez_i

#     # compute errors
#     ex = r[0] - y[5]
#     ey = r[1] - y[4]
#     ez = r[2] - y[3]

#     # error derivatives
#     ex_d = (ex - ex_prev)/Ts
#     ey_d = (ey - ey_prev)/Ts
#     ez_d = (ez - ez_prev)/Ts

#     # error integrals
#     ex_i += ex*Ts
#     ey_i += ey*Ts
#     ez_i += ez*Ts

#     taux = kpid(ex,ex_d,ex_i,gainsx)
#     tauy = kpid(ey,ey_d,ey_i,gainsy)
#     tauz = kpid(ez,ez_d,ex_i,gainsz)

#     # update derivative terms
#     ex_prev = ex
#     ey_prev = ey
#     ez_prev = ez

#     return [taux,tauy,tauz]

# def kpid(e,ed,ei,gains):
#     K,P,I,D = gains
#     u = K * (P*e + I*ei + D*ed)
#     return u 

# # LQR attitude controller
# def body_rate_lqr(x,phi,xd):
#     x = np.array(x)
#     xd = np.array(xd)

#     rho = 1
#     Q = np.diag([1,1,1])
#     R = rho*np.diag([1,1,1,1])

#     # get linearized dynamics
#     A_, B_ = A_omega(xd,phi), B_omega(phi)

#     # get LQR gains
#     K_,_,_ = lqr(A_,B_,Q,R)

#     # get control input
#     u = -K_ @ (x - xd)

#     return u

# # PID attitude controller
# def pid_control(x,u):
#     # get state variables
#     p,theta,v,omega,varphi = x[:3],x[3:6],x[6:9],x[9:12],x[12]

#     # get control input
#     c = 0.1 * (p[2]-X_ref[])

#     return u

# # terminal cost function parameters
# # cost function parameters
# t_x,t_y,t_z = 0,0,1
# t_dx,t_dy,t_dz = 0,0,1
# t_phi,t_th,t_psi = 0,0,0
# t_ox,t_oy,t_oz = 0,0,0
# t_varphi = 1

# Q_mat_terminal = beta * np.diag([t_x,t_y,t_z,t_psi,t_th,t_phi,t_dx,t_dy,t_dz,t_ox,t_oy,t_oz,t_varphi])
