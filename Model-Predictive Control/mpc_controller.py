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
    def __init__(self, waypoints, N=10, dt=0.1, x=0,y=0,th=0,v=0, L=3.0):
        self.waypoints=waypoints
        self.N = N          # Prediction horizon
        self.dt = dt        # Time step
        self.L = L          # Vehicle wheelbase
        self.max_acceleration = 1.0
        self.xw=1
        self.yw=1
        self.thetaw=0
        self.vw=0
        
        self.init_state=state(x,y,th,v)
        self.errorcount=0
        self.history=[0,0]
    
    
    def model(self, init_state, inputs):
        x = init_state[0]
        y = init_state[1]
        theta = init_state[2]
        v = init_state[3]
        throttle,delta=inputs
        
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + (v / self.L) * np.tan(delta) * self.dt
        v_next = v + throttle*self.max_acceleration * self.dt
        next_state=np.array([x_next, y_next, theta_next, v_next])
        
        return next_state
    
  
    
    def cost_function(self, b, *args):
        state,waypoints=args
        cost = 0.0
        for i in range(int(self.N)):
            u=np.array([b[i],b[i+self.N]])
            state= self.model(state,u)
            refx,refy=waypoints[i]
            cost += self.xw*(state[0]-refx)**2
            cost += self.yw*(state[1]-refy)**2
        cost=cost/2/2
        #print(cost)
        return cost
    
    def optimize_controls(self,gstate):
        closest_idx = np.argmin(np.sum((self.waypoints - gstate[:2])**2, axis=1))
        ref_path = self.waypoints[closest_idx:closest_idx + self.N]
        if len(ref_path) < self.N:
            remaining = self.N - len(ref_path)
            wraparound = self.waypoints[:remaining]
            ref_path = np.vstack((ref_path, wraparound))

        u0 = np.zeros((self.N*2))  # Initial guess: [throttle, steering]
        throttle_bounds = [(0, 1) for _ in range(self.N)]
        steering_bounds = [(-np.pi/4, np.pi/4) for _ in range(self.N)]
        bounds = throttle_bounds + steering_bounds
        result = minimize(
            self.cost_function,
            u0,
            args=(gstate, ref_path),
            bounds=bounds,
            method='SLSQP'
        )
        u=result.x
        if not result.success:
            print("Optimization failed:", result.message)

        #return u
        if result.success:
            print('sent')
            print(u)
            self.history.append([u[0],u[self.N]])
            return np.array([u[0],u[self.N]]) # Return optimized (throttle, steer) pairs
        else:
            print('error')
            self.errorcount=self.errorcount+1
            print(self.errorcount)
            return np.array(self.history[-1])  # Fallback if optimization fails