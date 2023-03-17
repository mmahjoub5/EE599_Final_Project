from vehicle import Driver
import math
from abc import ABC, abstractmethod

class DynamicsManager(ABC):
    @abstractmethod
    def dynamics():
        pass
    @abstractmethod
    def step():
        pass


'''
I inherited the Driver class to perform the dynamics of our car

'''
class CarDynamics(Driver):
    def __init__(self, v) -> None:
        super().__init__()
        self.car_node = super(CarDynamics,self).getFromDef('car')
        self.translationField = self.car_node.getField('translation')
        self.V = v
        self.omega = None
    
    def dynamics(self, x, y, theta):
        x_new =  x + self.V * math.sin(theta)
        y_new = y + self.V * math.cos(theta)
        theta_new = theta
        return x_new, y_new, theta_new
    
    def mpc():
        pass
    
    def step(self, theta):
        print("this is my log")
        position = self.car_node.getPosition()
        x = position[0]
        y = position[1]
        print(theta)
        x_new, y_new, theta_new = self.dynamics(x, y, theta)
        self.translationField.setSFVec3f([x_new, y_new, 0.1])
        return super(CarDynamics,self).step()
        
    def printPosition(self):
        print(self.car_node.getPosition())






