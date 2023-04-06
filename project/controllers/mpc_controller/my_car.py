import numpy as np
import math

class myCar:
    
    
    #properties: wRange, speed, dMax, dims

    #constructor 
    def __init__(self, x, y, theta, angRange, speed, dMax, dims):
        #x,y,z state values 
        self.x = x
        self.y = y
        self.theta = theta

        self.angRange = angRange
        self.speed = speed
        self.dMax = dMax
        self.dims = dims

        #self.V = V 
        #self.dt = dt


     
    def car_set_values(self):

        self.wRange = np.array([-(self.angRange), self.angRange])

        
        print("works")


        

    #dynamics 

    def dynamics(self, V, dt, control):
        x_new =  self.x + V * dt * (dt * math.cos(self.theta))
        y_new = self.y + V * dt *  (dt * math.sin(self.theta))
        theta_new =  self.theta + dt * control

        print("dynamics works")
        return x_new, y_new, theta_new



car = myCar(1, 1, 10, 1, 1, 1, 3)
car.car_set_values()

[x_new, y_new, theta_new] = car.dynamics(1, 1, 1)

print(x_new)



