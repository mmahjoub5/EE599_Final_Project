import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
from scipy.linalg import block_diag
from scipy.optimize import NonlinearConstraint, LinearConstraint, minimize, Bounds
from math import factorial
from gekko import GEKKO

def imageSubplot(data:list, title:list, showFig:bool=True):
        plt.figure()
        for i in range(len(data)):
            plt.subplot(1, len(data), i+1)
            plt.title(title[i])
            plt.plot(data[i])
        if showFig:
            plt.show()
        

class MPC(object):
    def __init__(self, speed, H, pos, dt, goalX, goalY) -> None:
        self.x_init = pos[0]
        self.y_init = pos[1]
        self.theta_init = pos[2]
        self.time_steps = H
        self.number_of_states =  3 * (H+1)
        self.number_of_controls = H
        self.num_decisions = self.number_of_states + self.number_of_controls
        self.X0 = np.zeros(self.num_decisions)
        self.X0[0] = self.x_init
        self.X0[H+1] = self.y_init
        self.X0[2*(H+1)] = self.theta_init
        self.H = H
        self.goalX = goalX
        self.goalY = goalY
        self.dt = dt
        self.v = speed
        self.c = self.constraints()
        self.N = self.X0.shape[0]
        self.Window = int((self.N-3)/4)
        self.m  = GEKKO()
        self.m.time = np.linspace(0,20,41)

    def setBounds(self):
        pass
  
    def xPosConstraint(self, x,i, ceq):
        ceq[i] = x[i+1] - x[i] - self.dt * self.v * np.cos(x[2*(self.Window+1) + i]) 
        ceq[self.H + i] = x[(self.Window +1) + i + 1] - x[self.Window + 1 + i] - self.dt * self.v * np.sin(x[2*(self.Window+1) + i])
        ceq[2*self.H + i] = (x[2*(self.Window +1) + i + 1] - x[2*(self.Window +1) + i] - self.dt * x[3*(self.Window+1) + i])
        
        x_initial = self.X0[0] - self.x_init * 0.5
        y_initial = self.X0[self.Window+1] - self.y_init * 0.5
        theta_intial = self.X0[2*self.Window +3] - self.theta_init * 0.5
        
        #x_initial, y_initial, theta_intial)

        ceq = np.vstack((ceq, x_initial, y_initial, theta_intial))
        ceq = ceq.flatten()
        # print(ceq.shape)
        #print(ceq)
        return ceq
    
    def constraintPos(self, x,):
        ceq = np.zeros((3*self.H,1))
        for i in range(self.time_steps):
            ceq[i] = x[i+1] - x[i] - self.dt * self.v * np.cos(x[2*(self.Window+1) + i]) 
            ceq[self.H + i] = x[(self.Window +1) + i + 1] - x[self.Window + 1 + i] - self.dt * self.v * np.sin(x[2*(self.Window+1) + i])
            ceq[2*self.H + i] = (x[2*(self.Window +1) + i + 1] - x[2*(self.Window +1) + i] - self.dt * x[3*(self.Window+1) + i])
            
        x_initial = self.X0[0] - self.x_init
        y_initial = self.X0[self.Window+1] - self.y_init
        theta_intial = self.X0[2*self.Window +3] - self.theta_init
        
        #x_initial, y_initial, theta_intial)

        ceq = np.vstack((ceq, x_initial, y_initial, theta_intial))
        ceq = ceq.flatten()

        return ceq

    def constraints(self):
        N = self.X0.shape[0]
        Window = int((N-3)/4)

        x_init = lambda x : x[0] - self.x_init
        y_init = lambda x : x[Window+1] - self.y_init
        theta_init = lambda x : x[2*Window + 3] - self.theta_init
       
        c = []
        scipy_c = []
       
        #scipy_c.append(NonlinearConstraint(self.xPosConstraint, 0, 0))
        scipy_c.append({'type': 'eq', 'fun': self.xPosConstraint})

        #cons_per_i = [{'type': 'eq', 'fun': self.xPosConstraint, 'args':(i,)} for i in range(self.time_steps)]
        ceq = np.zeros((3*self.H,1))
        cons_per_i = [{'type':'eq', 'fun': lambda z, i=i: self.xPosConstraint(z, i, ceq)} for i in range(self.time_steps)][-1]
        print(cons_per_i)
        #cons_per_i = [{'type':'eq', 'fun': self.constraintPos}]
        return cons_per_i
    
    def cost(self, x):
        c = np.sum((x[:self.H] - self.goalX) ** 2   + (x[self.H+1:2*self.H +1] - self.goalY) ** 2)
        return c
    
    def run(self):
        # prob = cp.Problem(self.objective, self.c)
        # prob.solve()
        # print(self.X0.value)
        #[-100*ones(H+1, 1); -100*ones(H+1, 1);-100*ones(H+1, 1); -wMax*ones(H, 1)];'
      
        bnds = -100* np.ones((self.time_steps+1,1))
        temp =  -100* np.ones((self.time_steps+1,1))
        temp2 =  -100* np.ones((self.time_steps+1,1))
        temp1 = -1.1* np.ones((self.time_steps,1))
        lbnds = np.vstack((bnds, temp2, temp, temp1))
      
        ubnds = -lbnds
        
        bounds = []
        for i in range (self.X0.shape[0]):
            bounds.append((lbnds[i], ubnds[i]))
        req = minimize(self.cost, self.X0, method='SLSQP', constraints=self.c, options={'disp': True}, bounds=bounds)
        print(req.x)

mpc_obj = MPC(0.1, 10, [-4, -4, 0], 0.5, goalX=10, goalY=10)
mpc_obj.run()
