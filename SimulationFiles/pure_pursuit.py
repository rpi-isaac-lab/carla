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
import csv

# TODO: Tune parameters of PID with these global variables
param_Kp = 4
param_Ki = 0
param_Kd = 4
# TODO: Tune parameters of Pure Pursuit with these global variables
param_K_dd = 0.4
# The above parameters will be used in the Carla simulation
# The simple simulation in tests/control/control.ipynb does not use these parameters
import carla
import argparse

def CONE(world):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """

    waypointroadids=waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv')[:5]
    waypointlaneids=waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv')[:5]
    waypointdistances=waypointfileProcessorfloat('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv')[:5]
    
    

    map = world.get_map()
    
    

    bloack_array = []

    for i in range(len(waypointroadids)):
        waypoint = map.get_waypoint_xodr(waypointroadids[i], waypointlaneids[i], waypointdistances[i])
        if waypoint:
            spawn = waypoint.transform
            bloack_array.append([spawn.location.x,spawn.location.y])
            
    return bloack_array

def waypointfileProcessorint(csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader, None)
            for row in reader:
                column_data.append(row)
            for i in range(len(column_data)):
                 column_data[i] = int(column_data[i][0])
        return column_data
       
def waypointfileProcessorfloat(csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader,None)
            for row in reader:
                column_data.append(row)
            for i in range(len(column_data)):
                 column_data[i]=float(column_data[i][0])
        return column_data
        

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')

    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    

class PurePursuit:
    def __init__(self, K_dd=param_K_dd, wheel_base=2.85, waypoint_shift=1.4):
        self.K_dd = K_dd #Parameter related to lookahead distance and tuned based on speed
        #Find Wheel base of the vehicle
        self.wheel_base = wheel_base
        self.waypoint_shift = waypoint_shift
        self.lane_shift= 0 #Change shift in lane, positive values right
        # argparser = argparse.ArgumentParser(description=__doc__)
        # argparser.add_argument('--host',metavar='H',default='127.0.0.1',help='IP of the host server (default: 127.0.0.1)')
        # argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,help='TCP port to listen to (default: 2000)')
        # args = argparser.parse_args()
        client = carla.Client('127.0.0.1', 2000)
        self.world = client.get_world()
        self.vehicle = None
        self.lanesensor = None
        self.laneinvade = False
        self.blocked_points = CONE(self.world)
    
    def _on_invasion(self,event):
        print(event.crossed_lane_markings)
        self.laneinvade = True
        return
    
    def get_control(self, waypoints, speed):
        # transform x coordinates of waypoints such that coordinate origin is in rear wheel
        waypoints[:,0] += self.waypoint_shift
        if self.vehicle == None or self.lanesensor == None:
            actor_list = self.world.get_actors()
            # Find the driver car
            for id in actor_list:
                try:
                    if id.attributes["role_name"] == "hero":
                        self.vehicle = id
                    elif id.attributes["role_name"] == "lanesensor":
                        self.lanesensor = id
                    else:
                        continue
                except:
                    continue
        Laneinvade=self.lanesensor.listen(lambda event: self._on_invasion(event))
        waypoint27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
        if ((waypoint27.lane_change==1 or waypoint27.lane_change ==3) and waypoint27.right_lane_marking.type == 1):
            self.lane_shift = 2
        elif not self.laneinvade:
            self.lane_shift = 0

        # t = self.vehicle.get_transform()
        # if self.lane_shift == 0:
        #     for point in self.blocked_points:
        #         if abs(t.location.x - point[0]) < 10 and abs(t.location.y - point[1]) < 10:
        #             print("WARNING OBJECT")
        #             if ((waypoint27.lane_change==2 or waypoint27.lane_change ==3) and waypoint27.left_lane_marking.type == 1):
        #                 self.lane_shift = -2
        #             elif not self.laneinvade:
        #                 self.lane_shift = 0
        
        waypoints[:,1]+=self.lane_shift
        current_vehicle_location = self.vehicle.get_location()
        
                    

        #Waypoint(Transform(Location(x=-65.136642, y=0.547766, z=0.000000), Rotation(pitch=0.000000, yaw=-359.705353, roll=0.000000)))

        #if self.lane_shift != 0:
        #    print("Trying to change lanes")
        #else:
        #    print("Moving Along")
        if self.laneinvade:
            self.laneinvade = False
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
        
        waypoints[:,1]-=self.lane_shift
        #print(current_vehicle_location)
        # if abs(current_vehicle_location.x + 63) < 10 and abs(current_vehicle_location.y) < 5:
        #     print(steer,alpha)
        # if steer < .45 and steer > .4:
        #     print(alpha)
        
        #Override for attempted sharp turns in intersections by controller, unknown source
        if steer > .4:
            if waypoint27.is_junction:
                # print(waypoint27.junction_id)
                # if waypoint27.junction_id == 861:
                

                # print("Override")
                
                return 0,0
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


