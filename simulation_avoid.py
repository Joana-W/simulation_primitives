import numpy as np
import matplotlib.pyplot as plt
from numpy import sqrt
from numpy.linalg import norm

# Initial conditions (when running the sweep script comment the following lines out)
a = 0.18 # goal x
b = 0.18 # goal y
x0 = -0.1 #initial position in x-direction
y0 = -0.2 #initial position in y-direction
x_dot0 = 0.0
y_dot0 = 0.0

#Parameters
K = 50.0  # spring constant [N/m]
M = 0.73  # mass [kg]
D = 14.2 # damping coefficient [Ns/m]

R = 0.2 #saftey bubble radius [m]
r_eq = 0.15 #equilbrium radius [m]


# Time parameters
timestep = 0.001
total_time = 10.0

# Function to compute hand repulsive force in x direction
def F_h_x(x, y, R, a, b, P, D_h, x_dot, y_dot):
        return P * R * np.cos(np.arctan2(y, x)) - P * x - D_h * x_dot


# Function to compute hand repulsive force in y direction
def F_h_y(x, y, R, a, b, P, D_h, x_dot, y_dot):
    return P * R * np.sin(np.arctan2(y, x)) - P * y - D_h * y_dot


# Function to compute acceleration in x-direction
def acceleration_x(x, y, R, a, b, P, D_h, x_dot, y_dot):
    if x ** 2 + y ** 2 <= R **2: #inside the safety bubble
        return -K / M * x - D / M * x_dot + K / M * a + F_h_x(x, y, R, a, b, P, D_h, x_dot, y_dot)/M 
    else:
        return -K / M * x - D / M * x_dot + K / M * a

# Function to compute acceleration in y-direction
def acceleration_y(x, y, R, a, b, P, D_h, x_dot, y_dot):
    if x ** 2 + y ** 2 <= R ** 2: #inside the safety bubble
        return  -K / M * y - D / M * y_dot + K / M * b + F_h_y(x, y, R, a, b, P, D_h, x_dot, y_dot)/M
    else:
        return -K / M * y - D / M * y_dot + K / M * b

# Finite difference solver
def solve(a,b,x0,y0,x_dot0,y_dot0,R,r_eq):
    initial_bubble = False
    inside_distance = 0 
    G = np.sqrt((a**2+b**2))
    r0 = np.sqrt((x0**2+y0**2))
    alpha = G/r0
    traj_length = 0 #length of the end-effector's trajectory
    C1 = 0 #initializing the integration constant C1, which needs to be calculated
    max_inside_distance = 0 #maximum inside distance [m] (if the end-effector enters the restriced space, inside_distance tells us by how much)

    P = K*(G+r_eq)/(R-r_eq) #saftey bubble stiffness [N/m]
    beta = (2*alpha + alpha**2)
    P_virtuell = K*np.sqrt(r_eq**2+r0**2*(beta))/(R-r_eq) #virtual safety bubble stiffness [N/m]

    D_h = np.sqrt(1+4*M*(K+P_virtuell)) -D #safety bubble damping
    
    num_steps = int(total_time / timestep)
    x = np.zeros(num_steps)
    y = np.zeros(num_steps)
    x_dot = np.zeros(num_steps)
    y_dot = np.zeros(num_steps)

    x[0] = x0
    y[0] = y0
    x_dot[0] = x_dot0
    y_dot[0] = y_dot0

    if x0**2+y0**2 < r_eq**2: #checks whether the end-effector is inside of the restriced space
        inside = True
        inside_distance = np.sqrt(x0**2+y0**2)-r_eq
    else:
        inside = False
    
    
    for i in range(1, num_steps):

        if (x[i]**2 + y[i]**2 < R**2) and not initial_bubble: #when the end-effector enters the safety bubble for the first time the safety bubble damping is calculated
            initial_bubble = True
            Bubble_activated = True #needed, to ensure that only the cases, where the safety bubble is actually used are saved when running the sweep-script
            x_dot_entry = x_dot[i] #velocity of the end-effector in x-direction when the end-effector enters the safety bubble 
            y_dot_entry = y_dot[i] #velocity of the end-effector in y-direction when the end-effector enters the safety bubble 
            C1 = -np.sqrt(x_dot_entry**2+y_dot_entry**2)*M + (D+D_h+1)/(R-r_eq)
            if C1 >= 0:
                D_h = D_h
            else:
                D_h = ((K+P)*(R+r_eq)**2+M*(x_dot[i - 1]**2+y_dot[i - 1]**2))/((R-r_eq)*np.sqrt(x_dot[i - 1]**2+y_dot[i - 1]**2)) - D

        x_dot_dot = acceleration_x(x[i - 1], y[i - 1], R, a, b, P, D_h, x_dot[i - 1], y_dot[i - 1]) #calculates the acceleration in x-direction
        y_dot_dot = acceleration_y(x[i - 1], y[i - 1], R, a, b, P, D_h, x_dot[i - 1], y_dot[i - 1]) #calculates the acceleration in y-direction
        
        x_dot[i] = x_dot[i - 1] + x_dot_dot * timestep #calculates the velocity in x-direction
        y_dot[i] = y_dot[i - 1] + y_dot_dot * timestep #calculates the velocity in y-direction

        x[i] = x[i - 1] + x_dot[i] * timestep #calculates the x-coordinate of the position
        y[i] = y[i - 1] + y_dot[i] * timestep #calculates the y-coordinate of the position
        
        traj_length = traj_length + np.sqrt((x[i]-x[i - 1])**2+(y[i]-y[i - 1])**2)
        if x[i]**2 + y[i]**2 < r_eq**2: #checks whether the end-effector is inside of the restricted sphere
            inside = True
            inside_distance = np.sqrt(x[i]**2+y[i]**2)-r_eq
            if np.abs(inside_distance) > np.abs(max_inside_distance):
                max_inside_distance = inside_distance #saves the maximum inside distance
    
    if np.abs(x[-1]-a)<0.003 and np.abs(y[-1]-b)<0.003: #checks whether the goal position is reached within a certain tolerance
        goal_reached = True
    else:
        goal_reached = False
    
    return x, y, inside, P , P_virtuell, D_h, Bubble_activated, max_inside_distance, C1, goal_reached

#From here until the end of the file must be commented out, when running the sweep script
# Plot trajectory
x_traj, y_traj, inside, P, P_virtuell, D_h, Bubble_activated, inside_distance, C1 = solve(a,b,x0,y0,x_dot0,y_dot0,R,r_eq) #calls solve function
plt.plot(x_traj, y_traj, 'b', label='Trajectory')
print(inside) #prints whether the end-effector was inside

# Plot equilibruim radius circle with radius r_eq around the origin
circle = plt.Circle((0.0, 0), r_eq, color='k', fill=False)
plt.gca().add_patch(circle)

# Plot point (a, b)
plt.plot(a, b, 'go', label='(a, b)')

# Plot x0
plt.plot(x0, y0, 'g+', label='x0')

# Plot final trajectory point
plt.plot(x_traj[-1], y_traj[-1], 'g+', markersize=30, label='final point')

# Plot circle with radius R around the origin
circle = plt.Circle((0.0, 0), R, color='r', fill=False)
plt.gca().add_patch(circle)


plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory in x-y Plane')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
