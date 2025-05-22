# -*- coding: utf-8 -*-
"""
Created on Mon Mar 24 17:45:33 2025

@author: tralss
"""

import numpy as np
from scipy.optimize import minimize
class state:
    def __init__(self, X=0, Y=0, TH=0, V=0, CTE=0, ETH=0):
        self.x = X
        self.y = Y
        self.th = TH
        self.v = V
        self.cte = CTE
        self.eth = ETH


class MPCController:
    def __init__(self,N=10, dt=.1,x=0,y=0,th=0,v=0, L=2.85):
        #elf.waypoints=waypoints
        self.N = N          # Prediction horizon
        self.dt = dt        # Time step
        self.L = L          # Vehicle wheelbase
        self.max_acceleration = 50
        self.xw=1		#.5
        self.yw=0		#.5
        self.ethw=0
        self.vw=1
        self.steerw=0
        self.des_vel=8.94#20mph
        
        self.init_state=state(x,y,th,v)
        self.errorcount=0
        self.history=[[0.0,0.0]]
    
    
    def model(self, state,inputs):
        x = state[0]
        y = state[1]
        theta = state[2]
        v = state[3]
        throttle,delta=inputs
        
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + (v / self.L) * np.tan(delta) * self.dt
        v_next = v + throttle*self.max_acceleration * self.dt
        next_state=np.array([x_next, y_next, theta_next, v_next])
        
        return next_state
    
  
    
    def cost_function(self, b, *args):
        state,waypoints=args
        print('ways')
        print(waypoints[0][0])
        cost = 0.0
        for i in range(int(self.N)):
            u=np.array([b[i],b[i+self.N]])
            state= self.model(state,u)
            #print(waypoints)
            print('waypoints')
            print(waypoints[i])
            print('x')
            print(state[0])
            print('y')
            print(state[1])
            refx,refy=waypoints[i]
            """
            if i < self.N - 1:
                dx_path = waypoints[i + 1][0] - waypoints[i][0]
                dy_path = waypoints[i + 1][1] - waypoints[i][1]
            else:
                dx_path = waypoints[i][0] - waypoints[i - 1][0]
                dy_path = waypoints[i][1] - waypoints[i - 1][1]
            
            desired_heading = np.arctan2(dy_path, dx_path)
            heading_error = state[2] - desired_heading
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
            """
            #cost += self.ethw*(heading_error)
            cost += self.xw*(state[0]-refx)**2
            cost += self.yw*(state[1]-refy)**2
            cost += self.steerw*(b[i+self.N]-self.history[-1][1])**2
            cost += self.vw * (state[3]-self.des_vel)**2
        cost=cost/2
        #print(cost)
        return cost
    
    def optimize_controls(self,gstate,ref_path):
        """
        closest_idx = np.argmin(np.sum((self.waypoints - gstate[:2])**2, axis=1))
        ref_path = self.waypoints[closest_idx:closest_idx + self.N]
        if len(ref_path) < self.N:
            remaining = self.N - len(ref_path)
            wraparound = self.waypoints[:remaining]
            ref_path = np.vstack((ref_path, wraparound))
        """

        gstate[2]=gstate[2]*np.pi/180		#Yaw inputs as degrees

        u0 = np.zeros((self.N*2))  # Initial guess: [throttle, steering]
        throttle_bounds = [(0, 1) for _ in range(self.N)]
        #steering_bounds = [(-np.pi/5, np.pi/5) for _ in range(self.N)]
        steering_bounds = [(-np.pi/12, np.pi/12) for _ in range(self.N)]
        bounds = throttle_bounds + steering_bounds
        result = minimize(
            self.cost_function,
            u0,
            args=(gstate, ref_path),
            bounds=bounds,
            method='SLSQP'
        )
        u=result.x
        return [u[0],u[self.N]]
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
            return None#Fallback if optimization fails
