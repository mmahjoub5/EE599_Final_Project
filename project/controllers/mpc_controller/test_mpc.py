from  gekko_mpc import MPC

import math

def dynamics(mpc:MPC, x, y, v, theta, control_u, control_v):
    x_new =  x + v * mpc.dt * (mpc.dt * math.cos(theta))
    y_new = y + v * mpc.dt *  (mpc.dt * math.sin(theta))
    theta_new =  theta + mpc.dt * control_u
    v_new = v + mpc.dt * control_v
    return x_new, y_new, theta_new, v_new

x_new = -40
y_new = -50
theta_new = 0 
H =3
mpc = MPC(H=H, goalX=0, goalY=0, x_init=x_new, y_init= y_new, theta_init= theta_new)
mpc.V = 10
mpc.dt = 0.1
v_new = 1
while True:
    control_u, control_v = mpc.run(x_new,y_new,theta_new, v_new)
    if control_u is None:
        break
    else:
        x_new, y_new, theta_new, v_new = dynamics(mpc, x_new, y_new, v_new, theta_new, control_u, control_v)
        print(x_new, y_new, theta_new, v_new)


# def dynamics(mpc:MPC, x, y, theta, control_u):
#     x_new =  x + mpc.V * mpc.dt * (mpc.dt * math.cos(theta))
#     y_new = y + mpc.V * mpc.dt *  (mpc.dt * math.sin(theta))
#     theta_new =  theta + mpc.dt * control_u
#     return x_new, y_new, theta_new

# x_new = 3
# y_new = 3
# theta_new = 0 
# H =10
# mpc = MPC(H=H, goalX=0, goalY=0, x_init=x_new, y_init= y_new, theta_init= theta_new)
# mpc.V = 1
# mpc.dt = 0.05
# v_new = 1
# while True:
#     control_u = mpc.run(x_new,y_new,theta_new)
#     if control_u is None:
#         break
#     else:
#         x_new, y_new, theta_new = dynamics(mpc, x_new, y_new, theta_new, control_u)
#         print(x_new, y_new, theta_new, v_new)

