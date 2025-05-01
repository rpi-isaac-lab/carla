# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 13:27:19 2024

@author: searsk
"""
"""
A module that impliments a pure pursuit PID controller.

More info here: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
Originial Code from here: https://github.com/thomasfermi/Algorithms-for-Automated-Driving/blob/master/code/solutions/control/pure_pursuit.py
"""
import numpy as np
from get_target_point import get_target_point

# TODO: Tune parameters of PID with these global variables
param_Kp = 4
param_Ki = 0
param_Kd = 4
# TODO: Tune parameters of Pure Pursuit with these global variables
param_K_dd = 0.4
# The above parameters will be used in the Carla simulation
# The simple simulation in tests/control/control.ipynb does not use these parameters


class PurePursuit:
    def __init__(self, K_dd=param_K_dd, wheel_base=2.85, waypoint_shift=1.4):
        self.K_dd = K_dd #Parameter related to lookahead distance and tuned based on speed
        #Find Wheel base of the vehicle
        self.wheel_base = wheel_base
        self.waypoint_shift = waypoint_shift
    
    def get_control(self, waypoints, speed):
        # transform x coordinates of waypoints such that coordinate origin is in rear wheel
        waypoints[:,0] += self.waypoint_shift
        # 3 is the minimum look ahead distance, 20 is the max lookahead distance
        look_ahead_distance = np.clip(self.K_dd * speed, 5,20)

        track_point = get_target_point(look_ahead_distance, waypoints)
        if track_point is None:
            return None, None

        alpha = np.arctan2(track_point[1], track_point[0])

        # Change the steer output with the lateral controller.
        steer = np.arctan((2 * self.wheel_base * np.sin(alpha)) / look_ahead_distance)

        # undo transform to waypoints 
        waypoints[:,0] -= self.waypoint_shift
        return steer, alpha


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None
    
    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error*self.Ki*dt
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/dt*self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term


class PurePursuitPlusPID:
    def __init__(self, pure_pursuit=PurePursuit(), pid=PIDController(param_Kp, param_Ki, param_Kd, 0)):
        self.pure_pursuit = pure_pursuit
        self.pid = pid

    def get_control(self,waypoints, speed, desired_speed, dt, cornering_mult=1):
        steer, alpha = self.pure_pursuit.get_control(waypoints, speed)
        #print(alpha)
        if alpha is None:
            self.pid.set_point = desired_speed
        else:
            self.pid.set_point = max(desired_speed*(1/(1+cornering_mult*abs(alpha))),5)
        a = self.pid.get_control(speed,dt)
        return a, steer
