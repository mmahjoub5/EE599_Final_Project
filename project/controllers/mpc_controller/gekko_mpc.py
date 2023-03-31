from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from math import factorial, pi
m = GEKKO()
m.time = np.linspace(0,20,41)

# Parameters
mass = 500
b = m.Param(value=50)
K = m.Param(value=0.8)
H = 100
time_steps = H
number_of_states =  3 * (H+1)
number_of_controls = H
num_decisions = number_of_states + number_of_controls
dt = 0.5
goalX, goalY = 0,0
# Manipulated variable
# p = m.MV(value=0, lb=0, ub=100)
p = m.Array(m.MV,(num_decisions), value=0)
N = num_decisions
Window = int((N-3)/4)

for i in range(len(p)):
    p[i].STATUS = 1  # allow optimizer to change
    p[i].DCOST = 0.1 # smooth out gas pedal movement
    p[i].DMAX = 20   # slow down change of gas pedal
    p[i].value = 0
p[0].value = -4
p[H+1].value = -4
p[2*(H+1)].value  = 0

# Controlled Variable
# v = m.CV(value=0)
v = m.Array(m.CV,(H),lb= -pi,ub=pi )


for i in range(len(v)):
    v[i].STATUS = 1  # add the SP to the objective
    # v[i].SP = 40     # set point
    # v[i].TR_INIT = 1 # set point trajectory
    # v[i].TAU = 5     # time constant of trajectory
    
x_init = m.Param(-4)
y_init = m.Param(-4)
theta_init = m.Param(0)
dt = m.Param(0.5)
V = m.Param(1)
#m.options.CV_TYPE = 2 # squared error
# Process model
cos = lambda x: 1 - (x**2)/2 
sin = lambda x: x - (x**3) / factorial(3)
eq = []
for i in range(H):
    print(p[i+1])
    eq.append(p[i+1]  == (p[i].value - dt * V * np.cos(p[2*(Window+1) + i].value)) )
    eq.append(p[(Window +1) + i + 1] == (p[Window + 1 + i].value - dt * V * np.sin(p[2*(Window+1) + i].value)))
    eq.append((p[2*(Window +1) + i + 1] == (p[2*(Window +1) + i].value - dt * p[3*(Window+1) + i].value)))

m.Equation(eq)
m.Equation(p[0] == 0)  
m.Equation(p[Window+1] == y_init)
m.Equation(p[2*Window+3] == theta_init * dt)

m.options.IMODE = 6 # control
m.Minimize(m.sum((p[:H] - goalX) ** 2   + (p[H+1:2*H +1] - goalY) ** 2))
m.solve(disp=True)

# # get additional solution information
# import json
# with open(m.path+'//results.json') as f:
#     results = json.load(f)

print(type(v[0]))
for i in range(H):
    print()
    # plt.figure()
    # plt.subplot(2,1,1)
    # plt.plot(m.time,p[i].value,'b-',label='MV Optimized')
    # plt.legend()
    # plt.ylabel('Input')
    # plt.subplot(2,1,2)
    # plt.plot(m.time,results['v1.tr'],'k-',label='Reference Trajectory')
    # plt.plot(m.time,v[i].value,'r--',label='CV Response')
    # plt.ylabel('Output')
    # plt.xlabel('Time')
    # plt.legend(loc='best')
    # plt.show()