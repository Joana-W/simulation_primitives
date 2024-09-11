import itertools
import numpy as np
import pandas as pd
#from simulation_avoid import solve, a, b, x0, y0, x_dot0, y_dot0, r_eq, R, P, D_h, P_virtuell, C_1
from simulation_hold import solve

#Definition of the condition for which the simulationis run
r_eq_range = np.linspace(0.075,0.2,3)
F_ext_x_range = [10,15,25] #external force in x-direction, could also be changed to a linspace
F_ext_y_range = [10,15,25] #external force in y-direction, could also be changed to a linspace

parameter_combinations = list(itertools.product(r_eq_range, F_ext_x_range, F_ext_y_range))

results = []

for combination in parameter_combinations:
    r_eq, F_ext_x, F_ext_y = combination
    x_traj, y_traj, outside, F_ext_x, F_ext_y, K, max_outside_distance = solve(r_eq, F_ext_x, F_ext_y)
    x_final = x_traj[-1]
    y_final = y_traj[-1]
            
    results.append([F_ext_x, F_ext_y, r_eq, x_final, y_final, K, outside, max_outside_distance])

columns = ["F_ext_x", "F_ext_y", "r_eq", "x_final", "y_final", "K", "outside", "max outside distance" ]
df = pd.DataFrame(results, columns=columns)

# Save the DataFrame to a CSV file
df.to_csv('results_sweep_hold.csv', index=False)
print(f"All simulations completed.")
