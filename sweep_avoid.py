import itertools
import numpy as np
import pandas as pd
#from simulation_avoid import solve, a, b, x0, y0, x_dot0, y_dot0, r_eq, R, P, D_h, P_virtuell, C_1
from simulation_avoid import solve

#Define the range for the initial conditions, goal positions and radii tested
a_range = np.linspace(0.15,0.3,3)
b_range = np.linspace(0.15,0.3,3)
x0_range = np.linspace(-0.3,0.1,3)
y0_range = np.linspace(-0.3,0.0,3)
x_dot0_range = [0, 2]
y_dot0_range = [0, 2]
R_range = [0.15,0.2,0.25]
r_eq_range = [0.05,0.1]
goal_reached = False

#calculate the parameter combinations
parameter_combinations = list(itertools.product(a_range, b_range, x0_range, y0_range, x_dot0_range,y_dot0_range,R_range,r_eq_range))

results = []

for combination in parameter_combinations:
    a,b,x0,y0,x_dot0,y_dot0,R,r_eq = combination

    if r_eq < R: #only run the simulatino if r_eq is smaller than R
        if x0**2 + y0**2 > r_eq**2: #only run the simulation if the end-effectors initial position is outside of the restricted space
            x_traj, y_traj, inside, P , P_virtuell, D_h, Bubble_activated, max_inside_distance, C1, goal_reached = solve(a,b,x0,y0,x_dot0,y_dot0,R,r_eq)
            x_final = x_traj[-1] #final x-coordinate
            y_final = y_traj[-1] #final y-coordinate
            
            results.append([a, b, x0, y0, x_dot0, y_dot0, R, r_eq, x_final, y_final, P, P_virtuell, D_h, inside, max_inside_distance, Bubble_activated, goal_reached, C1])

columns = ["a", "b", "x0", "y0", "x_dot0", "y_dot0", "R", "r_eq", "final_x", "final_y", "P", "P_virtuell", "D_h",  "inside", "max_inside_distance", "Bubble_activated", "goal reached", "C1"]
df = pd.DataFrame(results, columns=columns)

df.to_csv('results_sweep avoid.csv', index=False) # Save the DataFrame to a CSV file
print(f"All simulations completed.")

