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

        self.number_of_controls =  2* (H - 1)
        self.number_of_states =  4 * (H)
        self.goalX, self.goalY = goalX, goalY

        self.num_decisions = self.number_of_states + self.number_of_controls
        self.traj = [[self.x_initial],[self.y_initial],[self.theta_intial]]
        
    def run(self, x, y, theta, v):   
        
        # update trajectory
        self.traj[0].append(x)
        self.traj[1].append(y)
        self.traj[2].append(theta)
        if (self.stopCriteria(x,y)):
            self.plotTraj()
            return None
        

        m = GEKKO(remote=False)
        p = self.createVariables(m, x,y,theta, v)
        print(p)
        # pdb.set_trace()
        m.Equation(self.setConstraints(m, p, x, y, theta, v))
        m.Minimize(sum((p[0:self.H] - self.goalX) ** 2   + (p[self.H:2*self.H] - self.goalY) ** 2))
        print("we are here")
        m.options.MAX_ITER = 1000


        try:
            m.solve(disp=False) # solve
        except:
            print("error")
            self.plotTraj()
            return None
        print(p[4 * self.H].value[0])
        return p[3*self.H + 1].value[0], p[4 * self.H].value[0]

    def createVariables(self, m:GEKKO, x,y, theta, v):
        p = m.Array(m.MV,(self.num_decisions), value=0)
        for i in range(len(p)):
            p[i].STATUS = 1  # allow optimizer to change
            p[i].value = 0
            if i <  2* self.H:
                p[i].lower = -400
                p[i].upper = 400
            elif i < 4*self.H:
                p[i].lower = -2*pi
                p[i].upper = 2*pi
            else:
                p[i].lower = -5
                p[i].upper = -5

        p[0].value = x
        p[self.H].value = y
        p[2*(self.H)].value  = theta
        p[4*(self.H) - 1].value = v
        return p
    
    '''
    
        x_t_1 = x_t + dt * v_t * cos(theta)   
        y_t_1 = y_t + v_t * dt * sin(theta)
        theta_t_1  = theta_t + dt * omega
        v_t_1 = v_t * dt * a
        [x, y, theta, v, omega, a]
    '''
    # set up the constraints
    def setConstraints(self, m:GEKKO, p:GEKKO.MV, x,y, v, theta) -> list:
        eq = []
        x_init = m.Param(x)
        y_init = m.Param(y)
        theta_init = m.Param(theta)
        dt = m.Const(self.dt)
        # V = m.Const(self.V)
        #state constraints  
        for i in range(self.H - 1):
            ##pdb.set_trace()
            eq.append(p[i+1]  == p[i] + dt * p[4*self.H - 1 + i] * m.cos(p[2*(self.H) + i]) )                           # x state constraints
            eq.append(p[(self.H) + i + 1] == (p[self.H + i] + dt * p[4*self.H - 1 + i] * m.sin((p[2*(self.H) + i]))))   # y state constraints
            eq.append((p[2*(self.H) + i + 1] == (p[2*(self.H) + i] + dt * p[3*(self.H) + i])))                          # theta state constraints
            eq.append((p[4* self.H + i] == p[4* self.H -1+ i] + dt * p[5* self.H - 1+ i]))                                                        # v state constraints
        
        #intial conditions
        # pdb.set_trace()
       

        eq.append(p[0] == x_init)
        eq.append( p[self.H] == y_init)
        eq.append(p[2*(self.H)] == theta_init)
        eq.append(p[4*(self.H) - 1] == 1)
        print(eq)
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

    
    
