import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp


def imageSubplot(data:list, title:list, showFig:bool=True):
        plt.figure()
        for i in range(len(data)):
            plt.subplot(1, len(data), i+1)
            plt.title(title[i])
            plt.plot(data[i])
        if showFig:
            plt.show()

def mpc():

    ## TODO: ADD dynamics constraints and cost function
    '''
        constraints:
            1. cannot not be outside the road
            2. dynamics constraints 
            3. control constraints (tbd)
    '''
    a = 1
    b = 1
    q =1
    q_f =1 
    r = 50
    x_init = 1
    time_steps = 20
    number_of_states = 20
    number_of_controls = 19
    state_matrix = -a * np.eye(time_steps, number_of_states)
    control_matrix = -b * np.eye(time_steps, number_of_controls)
    
    for i in range(state_matrix.shape[0]):
        if i < state_matrix.shape[0] - 1:
            state_matrix[i][i+1] = 1

    Aeq = np.concatenate((state_matrix, control_matrix), axis=1)
    Beq = np.zeros((time_steps))
    x = cp.Variable(number_of_controls + number_of_controls + 1)

    constraint = [Aeq @ x == Beq, x[0] - x_init == 0]
    cost = 0

    for i in range(time_steps):
        print(i + time_steps - 2)
        cost += q * x[i] ** 2 + r * x[i + time_steps - 1] ** 2
    cost += q_f * x[38] ** 2

    objective = cp.Minimize(cost)
    prob = cp.Problem(objective, constraint)
    prob.solve()

    print("\nThe optimal value is", prob.value)
    print("A solution x is")

    x_final = x.value[:20]
    u_final = x.value[20:39]

    imageSubplot([x_final, u_final], ["MPC state", "MPC control"])
    # print(Aeq)

mpc()