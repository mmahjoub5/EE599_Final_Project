import numpy as np
from controller import Supervisor

# Define the size and resolution of the grid
GRID_SIZE = 100 # number of cells in each dimension
CELL_SIZE = 0.5 # size of each cell in meters
GRID_CENTER = np.array([0, 0]) # center of the grid in meters

# Create the occupancy grid
occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)

# Set up the Webots supervisor node
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

# Get the road node
roadObject = []


road = supervisor.getFromDef("ROAD")

# Main loop
while supervisor.step(time_step) != -1:
    # Get the positions of all objects in the simulation
    objects = supervisor.getFromDef("OBJECTS").getField("children")
    
    # Reset the occupancy grid
    occupancy_grid.fill(0)
    
    # Loop through all cells in the occupancy grid
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            # Calculate the position of the cell in meters
            pos = (np.array([x, y]) - (GRID_SIZE // 2)) * CELL_SIZE + GRID_CENTER
            
            # Check if the position is inside the road
            if road.isInBoundingBox(pos):
                occupancy_grid[y, x] = 1

    # Do something with the occupancy grid, such as path planning or obstacle avoidance
