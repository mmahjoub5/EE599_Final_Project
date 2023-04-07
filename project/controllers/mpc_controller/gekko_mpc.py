from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from math import pi
import math
import pdb
# m = GEKKO()
# Parameters
class MPC(object):
    def __init__(self, H:int, goalX, goalY) -> None:
        self.H = H
        self.x_initial, self.y_initial, self.theta_intial =  None, None, None
        self.dt = 0.05
        self.V = 100

        self.number_of_controls = H - 1
        self.number_of_states =  3 * (H)
        self.goalX = goalX
        self.goalY = goalY

        self.num_decisions = self.number_of_states + self.number_of_controls
        self.traj = [[],[],[]]
        
    def run(self, x, y, theta):
        # print("x", self.traj[0])
        # print("y", self.traj[1])
        # print("theta", self.traj[2])
        # pdb.set_trace()
        self.x_initial, self.y_initial, self.theta_intial = x, y, theta
       
        # update trajectory
        self.traj[0].append(self.x_initial)
        self.traj[1].append(self.y_initial)
        self.traj[2].append(self.theta_intial)

        m = GEKKO(remote=False)
        p = self.createVariables(m)
        m.Equation(self.setConstraints(m, p))
        m.Minimize(sum((p[0:self.H] - self.goalX) ** 2   + (p[self.H:2*self.H] - self.goalY) ** 2))
        m.solve(disp=False) # solve
        
        return p[3*self.H + 1].value[0]

    def createVariables(self, m:GEKKO):
        p = m.Array(m.MV,(self.num_decisions), value=0)
        for i in range(len(p)):
            p[i].STATUS = 1  # allow optimizer to change
            p[i].value = 0
            if i < self.number_of_states - (self.H):
                p[i].lower = -100
                p[i].upper = 100
            else:
                p[i].lower = -pi/2
                p[i].upper = pi/2

        p[0].value = self.x_initial
        p[self.H].value = self.y_initial
        p[2*(self.H)].value  = self.theta_intial 
        
        return p
    

    # set up the constraints
    def setConstraints(self, m:GEKKO, p:GEKKO.MV) -> list:
        eq = []
        dt = m.Const(self.dt)
        V = m.Const(self.V)
        #state constraints  
        for i in range(self.H - 1):
            ##pdb.set_trace()
            eq.append(p[i+1]  == p[i] + dt * V * m.cos(p[2*(self.H) + i]) )
            eq.append(p[(self.H) + i + 1] == (p[self.H + i] + dt * V * m.sin((p[2*(self.H) + i]))))
            eq.append((p[2*(self.H) + i + 1] == (p[2*(self.H) + i] + dt * p[3*(self.H) + i])))
            
        #intial conditions
        ##pdb.set_trace()
        eq.append(p[0] == self.x_initial)
        eq.append( p[self.H] == self.y_initial)
        eq.append(p[2*(self.H)] == self.theta_intial)
   
        return eq


    def plotTraj(self):
        plt.figure()
        plt.plot(self.traj[0], self.traj[1])
        plt.show()

    
    
