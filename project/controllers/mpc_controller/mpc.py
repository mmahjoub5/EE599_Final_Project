import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
from scipy.linalg import block_diag



def imageSubplot(data:list, title:list, showFig:bool=True):
        plt.figure()
        for i in range(len(data)):
            plt.subplot(1, len(data), i+1)
            plt.title(title[i])
            plt.plot(data[i])
        if showFig:
            plt.show()

class MPC(object):
    def __init__(self, speed, H, pos:tuple) -> None:
        self.x_init = pos[0]
        self.y_init = pos[1]
        self.time_steps = H
        self.number_of_states = H
        self.number_of_controls = H - 1
        self.V = speed

    def constraints():
        pass
    def cost():
        pass
    def run(self):
        pass
     
     
def mpc():

    ## TODO: ADD dynamics constraints and cost function
    '''
        constraints:
            1. cannot not be outside the road
                30 + (75 - 10.75) < x_pose < 30 + (75 + 10.75)
                30 + (75 - 10.75) < y_pose < 30 + (75 + 10.75)
            2. dynamics constraints 
                X - vsin(theta) = 0
                Y - viscose(theta) = 0
                Theta - omega = 0
                V - a = 0 (constant speed)
            3. control constraints (tbd)
                abs(omega) < pi /2 
                
        cost function:
            cost = sum(q * (x_pose^2 + y_pose^2) + r * omega^2) + q_f * (x_pose^2 + y_pose^2)
            
            from mpc example:
                f1 = sum((x(1:H+1) - goalX).^2) + sum((x(2*H+2:3*H+2) - goalY).^2);
        X:
            x_pose.Size = (H, 1) where H is horizon  
            y_pose.Size = (H, 1) where H is horizon 
            theta.Size = (H, 1) where H is horizon

            X = [x_pose; 
                y_pose; 
                theta]
    '''
    a = 1
    b = 1
    q =1
    q_f =1 
    r = 50
    x_init = 1
    y_init = 1
    time_steps = 3
    number_of_states = 5
    number_of_controls = 4
    dt = 0.5
    v = 0.1 # constant speed
    AeqX  = -1 * np.eye(time_steps, time_steps + 1)
    tempX = -dt * np.eye(time_steps)
    AeqX = np.concatenate((AeqX, tempX), axis=1)

    AeqY = -1 * np.eye(time_steps, time_steps + 1)
    tempY = -dt * np.eye(time_steps)
    AeqY = np.concatenate((AeqY, tempY), axis=1)

    BeqY = np.zeros((time_steps))
    BeqX = np.zeros((time_steps))

    for i in range(time_steps):
        AeqY[i][i+1] = 1
        AeqX[i][i+1] = 1

    
    Aeq = block_diag(AeqX, AeqY)
    Beq = np.concatenate((BeqX, BeqY), axis=0)

    print(Aeq)
    print(Aeq.shape)
    print(Aeq.shape)
    print(BeqX.shape)
    
    x = cp.Variable((2 * number_of_states) + number_of_controls)

    constraint = [
        Aeq @ x == Beq, 
        x[0] - x_init == 0, 
        x[number_of_states] - y_init == 0,
        x[:number_of_states] >= 30 + (75 - 10.75),
        x[:number_of_states] <= 30 + (75 + 10.75),
        x[number_of_states: 2 * number_of_states] >= 30 + (75 - 10.75),
        x[number_of_states: 2 * number_of_states] <= 30 + (75 + 10.75),
        x[2 * number_of_states:] <= np.pi / 2,
        x[2 * number_of_states:] >= -np.pi / 2,
    ]
    
    # cost = 0
    # for i in range(time_steps):
    #     print(i + time_steps - 2)
    #     cost += q * x[i] ** 2 + r * x[i + time_steps - 1] ** 2
        
    # cost += q_f * x[38] ** 2

    # objective = cp.Minimize(cost)
    # prob = cp.Problem(objective, constraint)
    # prob.solve()

    # print("\nThe optimal value is", prob.value)
    # print("A solution x is")

    # x_final = x.value[:20]
    # u_final = x.value[20:39]

    # imageSubplot([x_final, u_final], ["MPC state", "MPC control"])
    # print(Aeq)

mpc()