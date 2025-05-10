# -*- coding: utf-8 -*-
"""
MPC Controller for Carla Vehicle using Bicycle Kinematics
v. _
Created on Mon Mar 24 17:45:33 2025

@author: tralss
"""

import numpy as np
from scipy.optimize import minimize

class MPCController:
    """
    Controller Object
    """
    def __init__(self,N=10, dt=.1, L=2.85):
        """
        Intializes the Controller object and set weightages 
        and other parameters in the controller.

        Parameters
        ----------
        N : The length of the Prediction Horizon. 
        The default is 10.
        dt : Discrete Time step. The default is .1.
        L : Length of the wheelbase. The default is 2.85.

        Returns
        -------
        None.

        """
        #self.waypoints=waypoints       #commented out as waypoints sent directly from vehicle/system
        self.N = N                      # Prediction horizon
        self.dt = dt                    # Time step
        self.L = L                      # Vehicle wheelbase
        self.max_acceleration = 5.0     # Maximum acceleration to proportionalize throttle, arbitrarily set
         
        self.ethp=0                     #Records the The current orientation head error used for the next predicted cross track error
        
        #Weightages of each error, changing the weights prioritizes certain errors when trying to minimize overall error
        self.xw=1		               #Weightage of x axis coordinate error
        self.yw=1		               #Weightage of y axis coordinate error
        self.ethw=1                     #Weightage of the orientation error
        self.vw=1                       #Weightage of the velocity error to desired speed (Velocity control)
        self.steerw=2                   #Weightage of steer rate control
        self.des_vel=3      #8.94#20mph #Desired velocity parameter for velocity control
        self.ctew=1                     #Cross Track Error weightage
        
        #Weightages for parameters to minimize the Control Effort
        self.ctw=1                      #Weightage of the throttle control effort
        self.csw=2                      #Weightage of the steer control effort
        
        #Terms to track data in controller
        # - counts the number of times the optimization errors out
        # - tracks all the throttle and steer parameters sent out of the controller
        self.errorcount=0
        self.history=[[0.0,0.0]]
    
    def model(self, state,inputs):
        """
        Function that defines and implements the bicycle kinematics of the vehicle
        X, Y, orientation, and speed dynamic states modeled, in that order

        Parameters
        ----------
        state : The current state of the vehicle
        (either passed into the controller or the predicted 'current' state of a previous time horizon)
        inputs : The controller outputs to be optimized by error, throttle and steer angle

        Returns
        -------
        next_state : the predicted next state after one time step

        """
        #Takes in the current state and seperates them out into their corresponding variables
        x = state[0]
        y = state[1]
        theta = state[2]
        v = state[3]

        throttle,delta=inputs
        
        #Kinematics equations
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + (v / self.L) * np.tan(delta) * self.dt
        v_next = v + throttle*self.max_acceleration * self.dt
        
        #Puts the next state values into a numpy array
        next_state=np.array([x_next, y_next, theta_next, v_next])
        
        return next_state
    
    def fit_polynomial(self, waypoints):
        """
        Fits the N waypoints as a polynomial fit (fits to 1st, 2nd, or 3rd degree)
        *Note Global waypoints are used

        Parameters
        ----------
        waypoints : Waypoint subset 

        Returns
        -------
        None.

        """
        
        x_val = waypoints[:, 0]
        y_val = waypoints[:, 1]
        self.coeffs = np.polyfit(x_val, y_val, 1)       #Fit the x and y waypoints and save the coefficients as a object wide variable
        return None
    
    def cost_function(self, b, *args):
        """
        Function that defines and implements the bicycle kinematics of the vehicle
        X, Y, orientation, and speed dynamic states modeled, in that order

        Parameters
        ----------
        b : Design varaibles: Thottle and steer in a 2*N size row array (first N number of index is thottle values, second N number of index are steer values)
        state : The current state of the vehicle passed into the controller
        waypoints : waypoint subset used for analysis

        Returns
        -------
        cost : Value of the error or cost of the vehicle with respect to desired position

        """
        state,waypoints=args
        cost = 0.0      #Intialize cost
        self.fit_polynomial(waypoints)  #Set polynomial fit waypoint coefficient

        for i in range(int(self.N)):    #Loop through every time step in prediction horizon
            u=np.array([b[i],b[i+self.N]]) 
            state= self.model(state,u)  #Use current state of design variables and current state to model the next time step's state
            refx,refy=waypoints[i]
            
            #Find the difference of the predicted position to the corresponding waypoint position
            dx = state[0] - refx
            dy = state[1] - refy
            
            #Finds the difference or distance between one waypoint to the next
            if i < self.N - 1:
                dx_path = waypoints[i + 1][0] - waypoints[i][0]
                dy_path = waypoints[i + 1][1] - waypoints[i][1]
            else:
                dx_path = waypoints[i][0] - waypoints[i - 1][0]
                dy_path = waypoints[i][1] - waypoints[i - 1][1]
            
            #Difference used to determine the desired angle of orientation the vehicle should follow
            desired_heading = np.arctan2(dy_path, dx_path)
            
            f_x = np.polyval(self.coeffs, state[0]) #Create the polynomial function based on fitted waypoints
            
            #Most sources online seem to have some variation of this but doesn't work as well as desired_heading method I created above
            #theta_des = np.arctan(np.polyval(np.polyder(self.coeffs), state[0]))

            cte_next = (f_x - state[1]) + (state[3] * np.sin(self.ethp) * self.dt)  #Cross track error
            eth_next = (state[2] - desired_heading) + (state[3] * np.tan(b[i+self.N]) / self.L * self.dt) #oreintation (eth) error based on desired oreintation
            eth_next = np.arctan2(np.sin(eth_next), np.cos(eth_next))       #Normalize these values (shows better results)

            self.ethp=eth_next #Updates the eth global variable
            
            #Cost function
            cost += self.ctew*(cte_next-0)**2                           #Error in CTE, where desired error should be 0
            cost += self.ethw*(eth_next-0)**2                           #Error in ETH, where desired error should be 0
            cost += self.xw*(dx)**2                                     #Error in x position, where desired position is the correlated waypoint value
            cost += self.yw*(dy)**2                                     #Error in y position, where desired position is the correlated waypoint value
            cost += self.ctw * b[i]**2                                  #Minimize the control effort of the throttle value
            cost += self.csw * b[i+self.N]**2                           #Minimize the controle effort of the steer angle value
            cost += self.vw * (state[3]-self.des_vel)**2                #Velocity control set to the desired sped 
            cost += self.steerw*(b[i+self.N]-self.history[-1][1])**2    #Ensure the steer rate isn't too high to minimize sudden jerks in vehicle motion
            
        cost=cost/2/8 #Divide the cost by 2(number of parameters) for the asscoiated cost function
        #print(cost)
        return cost
    
    def optimize_controls(self,gstate,ref_path):
        """
        Optimizes the path of the vehicle to follow the waypoint subset based on throttle and steer.
        Returns the first optimized throttle and steer for the first time step. So if N=10, the
        optimiztion output ten throttle and steer values, one for each time step. This function
        only outputs the first pair.

        Parameters
        ----------
        gstate : The current state of the vehicle passed into the controller
        ref_path : The waypoint subset

        Returns
        -------
        list : The optimized throttle and steer angle values for the first time step analyzed

        """
        
        #Uncomment for Carla simulation
        #gstate[2]=gstate[2]*np.pi/180		#Yaw inputs as degrees
        
        gstate=gstate

        u0 = np.zeros((self.N*2))  # Initial guess: [throttle, steering]
        #Set bounds for dv optimization
        throttle_bounds = [(0, 1) for _ in range(self.N)]
        steering_bounds = [(-np.pi/5, np.pi/5) for _ in range(self.N)]
        #Below line used in Carla
        #steering_bounds = [(-np.pi/12, np.pi/12) for _ in range(self.N)]
        bounds = throttle_bounds + steering_bounds
        #Minimize function using scipy.optimize
        result = minimize(
            self.cost_function,
            u0,
            args=(gstate, ref_path),
            bounds=bounds,
            method='SLSQP'
        )
        u=result.x
        return [u[0],u[self.N]] #Returns the optimized throttle value and steer angle value
    
    #The follow statements not used as vehicle handles it
    #Essentially Controls kept constant, as if input throttle and steer angle remains if the optimization doesn't successfully happen
        if not result.success:
            print("Optimization failed:", result.message)

        #return u
        if result.success:
            print('sent')
            #print(u)
            self.history.append([u[0],u[self.N]])
            return np.array([u[0],u[self.N]]) # Return optimized (throttle, steer) pairs
        else:
            print('error')
            self.errorcount=self.errorcount+1
            print(self.errorcount)
            return np.array(self.history[-1])#Fallback if optimization fails
