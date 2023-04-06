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
        self.mpc = MPC(H=4, goalX= goalX, goalY = goalY)

    def dynamics(self, x, y, theta, control):
        x_new =  x + self.V * self.dt * (self.dt * math.cos(theta))
        y_new = y + self.V * self.dt *  (self.dt * math.sin(theta))
        theta_new =  theta + self.dt * control
        return x_new, y_new, theta_new
    
    def step(self, theta):
        print("x", self.x, "y" , self.y, "theta", self.theta)
        control = self.mpc.run(self.x, self.y, self.theta)
        print(control)
        self.x, self.y, self.theta = self.dynamics(self.x, self.y, self.theta, control)
        self.translationField.setSFVec3f([self.x, self.y, 0.0443397])
        return super(CarDynamics,self).step()
        
    def printPosition(self):
        print(self.car_node.getPosition())






