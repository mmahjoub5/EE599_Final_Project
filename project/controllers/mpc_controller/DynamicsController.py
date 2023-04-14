from vehicle import Driver
import math
from gekko_mpc import MPC
import numpy as np



'''
I inherited the Driver class to perform the dynamics of our car, this our bridge between our optimal controlle and weBots

'''
class CarDynamics(Driver):
    def __init__(self, V, dt, x_intial, y_initial, theta_initial, goalX, goalY) -> None:
        super().__init__()
        self.car_node = super(CarDynamics,self).getFromDef('car')
        self.translationField = self.car_node.getField('translation')
        self.V = V
        self.dt = dt

        self.x = x_intial
        self.y = y_initial
        self.theta = theta_initial
        self.translationField.setSFVec3f([x_intial, y_initial, 0.0443397])
        self.mpc = MPC(10, goalX=goalX, goalY= goalY, x_init=x_intial, y_init= y_initial, theta_init= 0)
        self.mpc.dt = dt
        self.mpc.V = V

    def dynamics(self, mpc:MPC, x, y, theta, control):
        x_new =  x + mpc.V * mpc.dt * (mpc.dt * math.cos(theta))
        y_new = y + mpc.V * mpc.dt *  (mpc.dt * math.sin(theta))
        theta_new =  theta + mpc.dt * control
        return x_new, y_new, theta_new
    
    def step(self, theta):
        
        control = self.mpc.run(self.x, self.y, self.theta)
        print(control)
        self.x, self.y, self.theta = self.dynamics(self.mpc, self.x, self.y, self.theta, control)
        self.translationField.setSFVec3f([self.x, self.y, 0.0443397])
        #change angle of car using quadcord rotation
        
        self.car_node.getField('rotation').setSFRotation([theta,-0.00213565,-0.0118727,0.999905])
        return super(CarDynamics,self).step()
        
    def printPosition(self):
        print(self.car_node.getPosition())






