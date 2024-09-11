# simulation_primitives

These are the files containing the two dimensional simulation used to verfiy the control parameters for the action primitives "Avoid" and "Hold" for my bachelor's thesis.

#### Avoid
The two dimensional simulation consists of a function solve(), which calls for each time step the function acceleration() and afterwards calculates the new position by doing a discrete integration. The function acceleration() calculates the acceleration from the forces acting. This function also checks whether the point is inside of the outer radius R around the predefined hand, and if so, it adds the forces triggered by the safety bubble. Afterwards the path of the end-effector gets shown in a plot
When running the sweep script, comment out the following parts in the file containing the simulation:
* The part, where the plot is created
* The part, where the initial conditions and the radii are defined
In the sweep script, one can define the different initial conditions and radii which should be tested. A table containing the results from all the cases is saved as a csv-file.

#### Hold
The two-dimensional simulation is similar to the one for the avoid case. But in this case the starting point is also the goal, and the for half of the time steps an external force trying to disturb the point is added in the acceleration()-function. Afterwards the force is set to zero. The point should then move back to the goal. In this case there are also no safety bubble forces acting since it is not needed to achieve the requirements. In the function solve() the needed stiffness-parameter is also calculated.
When running the sweep script, comment out the following parts in the file containig the simulation:
* The part, where the plot is created
* The part, where the external force and the radii are defined
In the sweep script, one can define the different external forces and radii which should be tested. A table containing the results from all the cases is saved as a csv-file.
