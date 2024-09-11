import numpy as np
import matplotlib.pyplot as plt
from numpy import sqrt
from numpy.linalg import norm
# Initial conditions
a = 0 # goal x
b = 0 # goal y
x0 = 0
y0 = 0
x_dot0 = 0
y_dot0 = 0

#Parameters
M = 0.73  # mass [kg]
D = 29 # damping coefficient [Ns/m]

#Comment out the following three lines when running the sweep-script
e_max= 0.1 #maximum distance to goal position [m]
F_ext_x = 25 #external force in x-direction [N]
F_ext_y = 25 #external force in y-direction [N]


# Time parameters
timestep = 0.001
total_time = 10.0

# Function to compute acceleration in x-direction
def acceleration_x(x, y, a, b, x_dot, y_dot, i, K,  F_ext_x, F_ext_y):
    if i < 5000:
        return -K / M * x - D / M * x_dot + K / M * a + F_ext_x/M
    else:
        return  -K / M * x - D / M * x_dot + K / M * a
    
    

# Function to compute acceleration in y-direction
def acceleration_y(x, y, a, b, x_dot, y_dot, i, K,  F_ext_x, F_ext_y):
    if i < 5000:
        return -K / M * y - D / M * y_dot + K / M * a + F_ext_y/M
    else:
        return  -K / M * y - D / M * y_dot + K / M * b 
    

# Finite difference solver
def solve(e_max, F_ext_x, F_ext_y):
    num_steps = int(total_time / timestep)

    K = np.sqrt(F_ext_x**2+F_ext_y**2)/e_max #stiffness [N/m]

    x = np.zeros(num_steps)
    y = np.zeros(num_steps)
    x_dot = np.zeros(num_steps)
    y_dot = np.zeros(num_steps)
    max_inside_distance = 0
    x[0] = x0
    y[0] = y0
    x_dot[0] = x_dot0
    y_dot[0] = y_dot0

    if x0**2+y0**2 > e_max**2: #checks whether the end-effector is inside of the restricted space (further away from the goal position than allowed)
        inside = True
        inside_distance = np.sqrt(x0**2+y0**2)-e_max
    else:
        inside = False
    
    for i in range(1, num_steps):        

        x_dot_dot = acceleration_x(x[i - 1], y[i - 1], a, b, x_dot[i - 1], y_dot[i - 1], i, K, F_ext_x, F_ext_y) #calculates the acceleration in x-direction
        y_dot_dot = acceleration_y(x[i - 1], y[i - 1], a, b, x_dot[i - 1], y_dot[i - 1], i, K, F_ext_x, F_ext_y) #calculates the acceleration in y-direction
        
        x_dot[i] = x_dot[i - 1] + x_dot_dot * timestep #calculates the velocity in x-direction
        y_dot[i] = y_dot[i - 1] + y_dot_dot * timestep #calculates the velocity in y-direction

        x[i] = x[i - 1] + x_dot[i] * timestep #calculates the x-coordinate of the end-effector
        y[i] = y[i - 1] + y_dot[i] * timestep #calculates the y-coordinate of the end-effector
        
        if x[i]**2 + y[i]**2 > e_max**2: #checks whether the end-effector is inside of the restricted space (further away from the goal position than allowed)
            inside = True
            inside_distance = np.sqrt(x[i]**2+y[i]**2)-e_max
            if np.abs(inside_distance) > np.abs(max_inside_distance):
                max_inside_distance = inside_distance #saves the maximum inside distance
        
        

    return x, y, inside, F_ext_x, F_ext_y, K, max_inside_distance

#From here until the end of the file must be commented out, when running the sweep script
# Plot trajectory
x_traj, y_traj, inside, F_ext_x, F_ext_y, K, max_inside_distance = solve(e_max, F_ext_x, F_ext_y)
plt.plot(x_traj, y_traj, 'b', label='Trajectory')
print(inside)

# Plot circle with radius e_max around the goal position
circle = plt.Circle((0, 0), e_max, color='k', fill=False)
plt.gca().add_patch(circle)

# Plot point (a, b)
plt.plot(a, b, 'go', label='(a, b)')

# Plot x0
plt.plot(x0, y0, 'g+', label='x0')

# Plot final trajectory point
plt.plot(x_traj[-1], y_traj[-1], 'g+', markersize=30, label='final point')


plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory in x-y Plane')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
