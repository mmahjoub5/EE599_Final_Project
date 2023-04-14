from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt  
from math import pi
import math
import pdb


class MPC(object):
    def __init__(self, goalX , goalY, H:int, x_init, y_init, theta_init) -> None:
        self.H = H
        self.x_initial, self.y_initial, self.theta_intial =  x_init, y_init, theta_init
        self.dt = 0.05
        self.V = 1

        self.number_of_controls = H - 1
        self.number_of_states =  3 * (H)
        self.goalX, self.goalY = goalX, goalY

        self.num_decisions = self.number_of_states + self.number_of_controls
        self.traj = [[self.x_initial],[self.y_initial],[self.theta_intial]]
        
    def run(self, x, y, theta):   
        # update trajectory
        self.traj[0].append(x)
        self.traj[1].append(y)
        self.traj[2].append(theta)
        if (self.stopCriteria(x,y)):
            self.plotTraj()
            return None
        

        m = GEKKO(remote=False)
        p = self.createVariables(m, x,y,theta)
       
        m.Equation(self.setConstraints(m, p, x, y, theta))
        m.Minimize(sum((p[0:self.H] - self.goalX) ** 2   + (p[self.H:2*self.H] - self.goalY) ** 2))
        print("we are here")
        m.options.MAX_ITER = 1000


        try:
            m.solve(disp=False) # solve
        except:
            print("error")
            self.plotTraj()
            return None
        
        return p[3*self.H + 1].value[0]

    def createVariables(self, m:GEKKO, x,y, theta):
        p = m.Array(m.MV,(self.num_decisions), value=0)
        for i in range(len(p)):
            p[i].STATUS = 1  # allow optimizer to change
            p[i].value = 0
            if i < self.number_of_states - (self.H):
                p[i].lower = -400
                p[i].upper = 400
            else:
                p[i].lower = - pi
                p[i].upper = pi

        p[0].value = x
        p[self.H].value = y
        p[2*(self.H)].value  = theta
        
        return p
    

    # set up the constraints
    def setConstraints(self, m:GEKKO, p:GEKKO.MV, x,y, theta) -> list:
        eq = []
        x_init = m.Param(x)
        y_init = m.Param(y)
        theta_init = m.Param(theta)
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
       

        eq.append(p[0] == x_init)
        eq.append( p[self.H] == y_init)
        eq.append(p[2*(self.H)] == theta_init)
   
        return eq


    def stopCriteria(self, x,y):
        if np.abs(x - self.goalX) < 0.4 and np.abs(y - self.goalY) < 0.4:
            return True
        else:
            return False
        
    def plotTraj(self):
        plt.figure()
        plt.plot(self.traj[0], self.traj[1])
        plt.show()

    
    
