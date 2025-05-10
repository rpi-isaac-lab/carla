import numpy as np
import matplotlib.pyplot as plt
#from mpc_controller import MPCController
from mpc_controller_new import MPCController


# Discrete-time bicycle model
def vehicle_model(state, control, dt, L=2.85):
    """
    Models the vehicle, using the bicycle kinematics model, at every timestep
    
    Parameters
    ----------
    state : Inputs the current state of the kinematics parameters:x,y,theta,speed
    control : Controller output to be determined: throttle and steer angle
    dt : Time step
    L : Length of vehicle, value (2.85m) googled based on vehicle used in Carla.

    Returns
    -------
    Numpy Array of the states after the time step using the controller outputs
    In the order of - X position, Y Position, Vehicle orientation (Theta), Vehicle Speed

    """
    x, y, theta, v = state
    #Prints controller inputs
    throttle=control[0]
    print('Throttle')
    print(throttle)
    delta = control[1]  # Throttle (0 to 1) and steering angle (radians)
    print('Steer')
    print(delta)
    
    # Vehicle acceleration
    max_acceleration = 5.0  # Max acceleration
    a = throttle * max_acceleration  # Convert throttle to acceleration using a proportion
    
    # Bicycle model equations
    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + (v / L) * np.tan(delta) * dt
    v_next = v + a * dt
    
    #Print states for analysis
    print('x')
    print(x_next)
    print('y')
    print(y_next)
    print('theta')
    print(theta_next)
    print('v')
    print(v_next)
    return np.array([x_next, y_next, theta_next, v_next])

# Define waypoints for MPC controller, defined differently
# Waypoint generations not used are commented out

#waypoints = np.array([[2, 2],[3,3], [4, 4],[5,5], [6, 6],[7,7], [8, 8],[9,9], [10, 10],[11,11],[12,12]])
#waypoints = np.array([[7,7],[10,10],[11,11], [12,12]])
#waypoints=np.array([[1,1]])
#waypoints = np.array([[i,i] for i in range(1, 151)])
#waypoints = np.array([[2,i] for i in range(1, 51)])
#waypoints = np.array([[i,3] for i in range(1, 51)])

def generate_circular_waypoints(radius=10, num_points=100, center=(0, 0)):
    """
    Generates waypoints in a perfect circle for the vehicle to follow

    Parameters
    ----------
    radius : The radius of the circle for the waypoints. The default is 10.
    num_points : The number of points desired. The default is 100.
    center : The origin of the circle The default is (0, 0).

    Returns
    -------
    waypoints : Numpy Array of generated waypoints in [[x,y],] form. Path follows a CW path, original theta must be 0 rad

    """
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    
    #Equations used to define the circle
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    x=-x                    #Way the x was defined was switched to accomodate for initial car orientation of 0 rad
    waypoints = np.column_stack((x, y))
    return waypoints

def generate_rounded_rectangle_waypoints(width=20, height=20, radius=5, num_points=100):
    """
    Generates waypoints in a rectangular shape with curved shape corners
    *Note: in its current state, non-smooth shape causes controller issues
    *Note: Generative AI assistance used for this function

    Parameters
    ----------
    width : The width of the rectangle in units. The default is 20.
    height : The height of the rectangle in units. The default is 20.
    radius : Radius of the curved corners The default is 5.
    num_points : The number of waypoints for generation. The default is 100.

    Returns
    -------
    waypoints: Numpy Array of generated waypoints in [[x,y],] form. Path follows a CCW path, orginal theta must be pi rad

    """
    #define the corners
    corners = [
        (width / 2 - radius, height / 2 - radius),
        (-width / 2 + radius, height / 2 - radius),
        (-width / 2 + radius, -height / 2 + radius),
        (width / 2 - radius, -height / 2 + radius)
    ]

    path = []
    arc_points = num_points // 4 // 2
    straight_points = num_points // 4 - arc_points
    
    #Generate the straight lines with the curved corners in sequential order 
    path += [(x, height / 2) for x in np.linspace(corners[0][0], corners[1][0], straight_points)]
    angles = np.linspace(0, np.pi / 2, arc_points)
    path += [(corners[1][0] - radius * np.sin(a), corners[1][1] + radius * np.cos(a)) for a in angles]
    path += [(-width / 2, y) for y in np.linspace(corners[1][1], corners[2][1], straight_points)]
    angles = np.linspace(np.pi / 2, np.pi, arc_points)
    path += [(corners[2][0] - radius * np.sin(a), corners[2][1] + radius * np.cos(a)) for a in angles]
    path += [(x, -height / 2) for x in np.linspace(corners[2][0], corners[3][0], straight_points)]
    angles = np.linspace(np.pi, 3 * np.pi / 2, arc_points)
    path += [(corners[3][0] - radius * np.sin(a), corners[3][1] + radius * np.cos(a)) for a in angles]
    path += [(width / 2, y) for y in np.linspace(corners[3][1], corners[0][1], straight_points)]
    angles = np.linspace(3 * np.pi / 2, 2 * np.pi, arc_points)
    path += [(corners[0][0] - radius * np.sin(a), corners[0][1] + radius * np.cos(a)) for a in angles]

    return np.array(path)

#waypoints = generate_circular_waypoints()
waypoints = generate_rounded_rectangle_waypoints()

# Initial state: x, y, theta, v
state = np.array([0.0, 10.0, np.pi, 1.0])
#^For all waypoint setups except the rectangular one, the theta term should be 0 and not pi

#Initialize the controller
dt=0.1
N=10
mpc = MPCController(N, dt=dt) 

def get_closest_waypoints(current_position, waypoints, N=10):
    """
    Pulls from the entire array of generated waypoints and finds the closet one, 
    and the next N-1 points in the array order
    Waypoints are cycled at the end of the array, so assumes cyclical/circular path

    Parameters
    ----------
    current_position: The current (x,y) position of the vehicle
    waypoints: Numpy Array of generated waypoints in [[x,y],] form
    N: The Prediction Horizon used for MPC

    Returns
    -------
    waypoints: A small subset of waypoints used for MPC analysis in the instance
    
    """
    
    dists = np.linalg.norm(waypoints - current_position[:2], axis=1) #Creates an array of normal lengths of vehicle to the desired position(all the waypoints)
    closest_idx = np.argmin(dists)                      #From the array, selects the index with the smallest distance 
    total_points = waypoints.shape[0]               
    
    # Wrap around if reaching the end
    indices = [(closest_idx + i) % total_points for i in range(N)]      #Specify the indicies of entire waypoint set used to pass through
    return waypoints[indices]
    
    


# Simulation
T = 20  # Total simulation 'time'


# Store state outputs for plotting purposes
trajectory = [state]

time=0      #Simulation starts at 0 seconds

controller=[]
while(time<T):
    waypoint_min = get_closest_waypoints(state,waypoints,N)      #Obtain waypoint subset
    #print(waypoint_min)
    #print(state)
    controls=mpc.optimize_controls(state,waypoint_min)              #Use MPC to recieve optimal vehicle inputs (Same thing as controller output)
    #controls=np.array([0.9,1])         #Hard code controls for testing
    
    #If the optimization fails, revert back to the last controller inputs, as if the vehicle is going the same way its been going
    if np.isnan(controls[0]):
        controls[0]=controller[-1][0]
    if np.isnan(controls[1]):
        controls[1]=controller[-1][1]
        
    controller.append([float(controls[0]),float(controls[1])])  #Save controller output data
    state=vehicle_model(state, controls, dt)        #Send the current vehicle state and vehicle input to find the new vehicle state after a time step with these controls
    trajectory.append(state)        #Save the state data in an array
    time=time+dt                    #Update sim for the next time step
    print(time)


trajectory = np.array(trajectory)   #Convert state data to numpy array

# Plot results
plt.figure
plt.plot(trajectory[:, 0], trajectory[:, 1], 'bo-', label='Trajectory')
plt.scatter(waypoints[:, 0], waypoints[:, 1], c='r', marker='x', label='Waypoints')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Vehicle Motion and MPC Controller Trajectory')
plt.legend()
plt.grid()
plt.show()
print('Controller outputs (throttle, steer)')
print(controller)

print('Trajectory (x,,y,theta,v)')
print(trajectory)

#Plot the waypoints alone
# plt.figure
# plt.scatter(waypoints[:, 0], waypoints[:, 1], c='r', marker='x', label='Waypoints')
# plt.xlabel('X Position')
# plt.ylabel('Y Position')
# plt.title('Waypoint Mapping')
# plt.legend()
# plt.grid()
# plt.show()
