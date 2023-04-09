from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from math import factorial, pi
import math
import pdb
m = GEKKO()
m.time = np.linspace(0,20,41)

# Parameters
mass = 500
b = m.Param(value=50)
K = m.Param(value=0.8)
H = 4
time_steps = H
number_of_states =  3 * (H)
number_of_controls = H - 1
num_decisions = number_of_states + number_of_controls
x_intial, y_initial, theta_intial = -1, 1, 0

dt = 0.05
goalX, goalY = 1,-1
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

    if (stopCriteria(x_intial,y_initial)):
        break

    traj[0].append(x_intial)
    traj[1].append(y_initial)
    traj[2].append(theta_intial)

    # Manipulated variable
    p = m.Array(m.MV,(num_decisions), value=0)
    N = num_decisions
    Window = int((N-3)/4)

    for i in range(len(p)):
        p[i].STATUS = 1  # allow optimizer to change
        p[i].value = 0
        if i < number_of_states - (H):
            p[i].lower = -100
            p[i].upper = 100
        else:
            p[i].lower = -pi/4
            p[i].upper = pi/4
    
  
    p[0].value = x_intial
    p[H].value = y_initial
    p[2*(H)].value  = theta_intial 

    print(p)
    
    x_init = m.Param(x_intial)
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
   
    
    try:
        q_f = 1
        m.Minimize(sum((p[0:H] - goalX) ** 2   + (p[H:2*H] - goalY) ** 2))
        x = m.solve(disp=False) # solve
        ###pdb.set_trace()
        print(p[number_of_states].value[0])
        print(p)
        print()
        print()
        print(p[number_of_states:])
        x_intial, y_initial, theta_intial = nextStep(p[3*H + 1].value[0], 1, x_intial, y_initial, theta_intial)
        print(x_intial, y_initial, theta_intial, )
        ###pdb.set_trace()
    except:
        print("Error")
        break

    m = GEKKO()

plt.figure()
plt.plot(traj[0], traj[1], 'ro')
plt.show()

# get additional solution information
import json
with open(m.path+'//results.json') as f:
    results = json.load(f)
