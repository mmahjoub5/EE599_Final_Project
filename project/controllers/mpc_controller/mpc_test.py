from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from math import pi
import math
import pdb
m = GEKKO(remote=False)
H = 4
time_steps = H
number_of_states =  3 * (H)
number_of_controls = H - 1
num_decisions = number_of_states + number_of_controls
x_initial, y_initial, theta_intial = -4, -4, 0

dt = 0.05
goalX, goalY = -1,-2
traj = [[],[],[]]

def nextStep(control,V, x,y,theta):
    x_new = x + 0.05 * V * math.cos(theta)
    y_new = y + 0.05 * V * math.sin(theta)
    theta_new = theta + 0.05 * control
    return x_new, y_new, theta_new

def stopCriteria(x,y):
    if np.abs(x - goalX) < 0.05 and np.abs(y - goalY) < 0.05:
        return True
    else:
        return False
while True:
    print("loop")
    if (stopCriteria(x_initial,y_initial,)):
        break
    traj[0].append(x_initial)
    traj[1].append(y_initial)
    traj[2].append(theta_intial)

    # Manipulated variable
    p = m.Array(m.MV,(num_decisions), value=0)
    N = num_decisions


    for i in range(len(p)):
        p[i].STATUS = 1  # allow optimizer to change
        p[i].value = 0
        if i < number_of_states - (H):
            p[i].lower = -100
            p[i].upper = 100
        else:
            p[i].lower = -pi/2
            p[i].upper = pi/2
    
  
    p[0].value = x_initial
    p[H].value = y_initial
    p[2*(H)].value  = theta_intial 

    x_init = m.Param(x_initial)
    y_init = m.Param(y_initial)
    theta_init = m.Param(theta_intial)
    dt = m.Const(0.05)
    V = m.Const(1)
    
    eq = []
    #pdb.set_trace()
    #state constraints

    for i in range(H - 1):
        ##pdb.set_trace()
        eq.append(p[i+1]  == p[i] + dt * V * m.cos(p[2*(H) + i]) )
        eq.append(p[(H) + i + 1] == (p[H + i] + dt * V * m.sin((p[2*(H) + i]))))
        eq.append((p[2*(H) + i + 1] == (p[2*(H) + i] + dt * p[3*(H) + i])))
        
    
     #intial conditions
    ##pdb.set_trace()
    eq.append(p[0] == x_init)
    eq.append( p[H] == y_init)
    eq.append(p[2*(H)] == theta_init)
    m.Equation(eq)
   
    
    #pdb.set_trace()
    q_f = 1
    m.Minimize(sum((p[0:H] - goalX) ** 2   + (p[H:2*H] - goalY) ** 2))
    x = m.solve(disp=False) # solve
    ###pdb.set_trace()
    # print(p[number_of_states].value[0])
    # print(p)
    # print(p[number_of_states:])

    x_initial, y_initial, theta_intial = nextStep(p[3*H + 1].value[0], 1, x_initial, y_initial, theta_intial)
    print(x_initial, y_initial, theta_intial, )
    ###pdb.set_trace()
    
    # stop loop from keyboard input 
    
    
    
    m = GEKKO()
# get additional solution information

plt.figure()
plt.plot(traj[0], traj[1], 'ro')
plt.show()
# import json
# with open(m.path+'//results.json') as f:
#     results = json.load(f)
