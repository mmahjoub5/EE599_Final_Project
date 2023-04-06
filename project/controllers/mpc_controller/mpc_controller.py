# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""vehicle_driver controller."""


import math
from vehicle import Driver
from controller import *
from DynamicsController import CarDynamics

driver = CarDynamics(V=100, dt=0.05, x_intial=-48.1024, y_initial=44.2733, theta_initial=0, goalX=-2, goalY=-1)

while True:
    omega = math.pi/4
    w = driver.step(omega)
    if (w == -1):
        break
    #print("not zero")
    # print(car_node.getPosition())
    driver.printPosition()
   
