# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 13:27:19 2024

@author: searsk
"""

"""
A module that implements a Pure Pursuit + PID controller.

More info here: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
Original Code from: https://github.com/thomasfermi/Algorithms-for-Automated-Driving/blob/master/code/solutions/control/pure_pursuit.py
"""

import numpy as np
from .get_target_point import get_target_point

# Global PID parameters (to be tuned)
param_Kp = 2
param_Ki = 0
param_Kd = 0

# Pure Pursuit parameter (tune based on vehicle dynamics/simulation)
param_K_dd = 0.4

class PurePursuit:
    def __init__(self, K_dd=param_K_dd, wheel_base=2.85, waypoint_shift=1.4):
        """
        Initializes the Pure Pursuit controller.
        
        Parameters:
        - K_dd: gain for look-ahead distance (proportional to speed)
        - wheel_base: distance between front and rear axles
        - waypoint_shift: adjustment to shift coordinate origin to rear axle
        """
        self.K_dd = K_dd
        self.wheel_base = wheel_base
        self.waypoint_shift = waypoint_shift
    
    def get_control(self, waypoints, speed):
        """
        Computes steering control using Pure Pursuit algorithm.

        Parameters:
        - waypoints: Nx2 array of (x, y) waypoints
        - speed: current speed of the vehicle

        Returns:
        - steer: steering angle command
        """
        # Shift waypoints to rear axle frame
        waypoints[:, 0] += self.waypoint_shift
        
        # Compute dynamic lookahead distance based on speed
        look_ahead_distance = np.clip(self.K_dd * speed, 3, 20)

        # Find the target point on the path
        track_point = get_target_point(look_ahead_distance, waypoints)
        if track_point is None:
            return 0

        # Compute angle to target point
        alpha = np.arctan2(track_point[1], track_point[0])

        # Calculate steering angle using bicycle model geometry
        steer = np.arctan((2 * self.wheel_base * np.sin(alpha)) / look_ahead_distance)

        # Revert waypoint transformation
        waypoints[:, 0] -= self.waypoint_shift
        return steer


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point):
        """
        Initializes a PID controller.

        Parameters:
        - Kp: proportional gain
        - Ki: integral gain
        - Kd: derivative gain
        - set_point: desired value the controller tries to reach
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None
    
    def get_control(self, measurement, dt):
        """
        Computes the control action using PID control.

        Parameters:
        - measurement: current measured value
        - dt: time step since last control update

        Returns:
        - control output (e.g., acceleration)
        """
        error = self.set_point - measurement
        self.int_term += error * self.Ki * dt
        if self.last_error is not None:
            self.derivative_term = (error - self.last_error) / dt * self.Kd
        self.last_error = error

        return self.Kp * error + self.int_term + self.derivative_term


class PurePursuitPlusPID:
    def __init__(self, pure_pursuit=PurePursuit(), pid=PIDController(param_Kp, param_Ki, param_Kd, 0)):
        """
        Combines Pure Pursuit (for steering) and PID (for speed) control.
        
        Parameters:
        - pure_pursuit: an instance of PurePursuit
        - pid: an instance of PIDController
        """
        self.pure_pursuit = pure_pursuit
        self.pid = pid

    def get_control(self, waypoints, speed, desired_speed, dt):
        """
        Computes both throttle/brake (acceleration) and steering commands.

        Parameters:
        - waypoints: Nx2 array of waypoints
        - speed: current speed of vehicle
        - desired_speed: target speed
        - dt: time step

        Returns:
        - a: acceleration command
        - steer: steering command
        """
        self.pid.set_point = desired_speed
        a = self.pid.get_control(speed, dt)         # Longitudinal control (throttle/brake)
        steer = self.pure_pursuit.get_control(waypoints, speed)  # Lateral control (steering)
        return a, steer

