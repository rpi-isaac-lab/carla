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
import time
import math
import map_config

def CONE(world):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """

    waypointroadids=np.loadtxt('/home/labstudent/carla/PythonAPI/isacc_gh_testing/Data/ExactObjectWaypointsRoadIDs.csv',int,skiprows=1)
    waypointlaneids=np.loadtxt('/home/labstudent/carla/PythonAPI/isacc_gh_testing/Data/ExactObjectWaypointsLaneIDs.csv',int,skiprows=1)
    waypointdistances=np.loadtxt('/home/labstudent/carla/PythonAPI/isacc_gh_testing/Data/ExactObjectWaypointsS.csv',float,skiprows=1)
    
    map = world.get_map()

    bloack_array = []

    for i in range(len(waypointroadids)):
        # if not i in [4,8,11,12,13]:
        waypoint = map.get_waypoint_xodr(int(waypointroadids[i]), int(waypointlaneids[i]), float(waypointdistances[i]))
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
        self.lane_change= False #Change shift in lane, positive values right
        self.lane_id= None #Change shift in lane, positive values right
        # argparser = argparse.ArgumentParser(description=__doc__)
        # argparser.add_argument('--host',metavar='H',default='127.0.0.1',help='IP of the host server (default: 127.0.0.1)')
        # argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,help='TCP port to listen to (default: 2000)')
        # args = argparser.parse_args()
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.vehicle = None
        self.lanesensor = None
        self.laneinvade = False
        self.blocked_points = CONE(self.world)
        self.blocked_points_index = [[4, 3, 17, 11], [2, 19, 0, 5], [24, 22, 14, 6], [7, 16, 25, 21]]
        

        self.lapcount=0
        self.map = self.world.get_map()
        self.vehicle_spawn_loc=self.map.get_waypoint_xodr(map_config.vehicle_spawn_roadid,map_config.vehicle_spawn_laneid,map_config.vehicle_spawn_S).transform.location
        self.current_blocked = [4,3,17,11]
        self.spawn_index = -5  
        self.simtime = time.time()
        self.new_object_feild = True
        self.lane_change_direction = -1

        self.lane_change = False
        self.lane_goal = 'Right'

        self.near_obj = 0
        self.unknown_obj = [11,3,2,0,24,6,25,16]
        
        #self.reset(object_type, spawn_index)     
        
        
    def update_lane_change(self,lane_change_direction = -1):
        if lane_change_direction == 0:
            #Stop Changing Lanes
            self.lane_change = False
        else:
            #Activate Changing Lanes
            self.lane_change = True


            #Correct direction
            self.lane_change_direction = lane_change_direction
            if lane_change_direction == 1:
                self.lane_goal = 'Right'
            else:
                self.lane_goal = 'Left'

        self.start_lanechange = time.time()

        return
    
    def waypointfileProcessorint(self,csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader, None)
            for row in reader:
                column_data.append(row)
            for i in range(len(column_data)):
                column_data[i] = int(column_data[i][0])
        return column_data
        
    
    def waypointfileProcessorfloat(self,csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader,None)
            for row in reader:
                column_data.append(row)
            for i in range(len(column_data)):
                 column_data[i]=float(column_data[i][0])
        return column_data
        
        
    def update_lapcount(self):
        current_pos = self.vehicle.get_location()
        if current_pos.distance(self.vehicle_spawn_loc) < 10.0 and time.time()-self.simtime > 5:
            self.lapcount += 1
            
            self.current_blocked = self.blocked_points_index[int((self.lapcount-1)%4)]
            self.simtime=time.time()#Change to change the time it roughly takes to get out of the start zone
        
        self.start_time = time.time()
    
    def _on_invasion(self,event):
        self.laneinvade = True
        return
    
    def get_control(self, waypoints, speed,obj,over_distance):
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
        self.look_ahead_distance = np.clip(self.K_dd * speed, 5,10) #Decrease lookahead from 20
        
        try:
            #Is this prone to throwing errors? Why is it in an except
            self.update_lapcount()
        except:
            cat = 0 
            
        if obj:
            if not self.lane_change:
                self.lane_shift = self.lane_change_eval(over_distance)

        steer_change = 0
        try:
            if self.lane_change:
                if self.lane_goal != self.what_lane_am_I_in():
                    self.lane_shift = self.lane_change_direction *2
    
                    #print("Trying to change lanes")

                else:
                    self.lane_shift = 0
                    self.update_lane_change(0)
                    


        except Exception as e:
            print(e)
        # print(self.lane_shift)
        waypoints[:,1]+=self.lane_shift

        

        # current_vehicle_location = self.vehicle.get_transform()
        
                    

        #Waypoint(Transform(Location(x=-65.136642, y=0.547766, z=0.000000), Rotation(pitch=0.000000, yaw=-359.705353, roll=0.000000)))

        #if self.lane_shift != 0:
        #    print("Trying to change lanes")
        #else:
        #    print("Moving Along")
        if self.laneinvade:
            self.laneinvade = False
        # 3 is the minimum look ahead distance, 20 is the max lookahead distance
        look_ahead_distance = np.clip(self.K_dd * speed, 5,20)
        
        
        nwp = self.map.get_waypoint(self.vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving)) # Nearest waypoint to vehicle
        #print("laneid", self.what_lane_am_I_in())
        track_point = get_target_point(look_ahead_distance, waypoints)
        
        if track_point is None:
            return None, None

        
        # t = self.vehicle.get_transform()
        
        alpha = np.arctan2(track_point[1], track_point[0])
        
        # Change the steer output with the lateral controller.
        steer = np.arctan((2 * self.wheel_base * np.sin(alpha)) / look_ahead_distance)

        # undo transform to waypoints 
        waypoints[:,0] -= self.waypoint_shift
        
        waypoints[:,1]-=self.lane_shift
        
       
        t = self.vehicle.get_transform()
        
        #Hardcoded troublemakers
        if nwp.road_id == 1933 and steer < 0:
            return -steer, alpha
        if steer > .4:
            waypoint27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
            # print(waypoint27.road_id)
            
            if waypoint27.is_junction:               

                # print("Override")
                
                return 0,0
        
        
        return steer, alpha
    
    def what_lane_am_I_in(self):

        
        w27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))

        #Correct directions
        if str(w27.right_lane_marking.type) == 'Solid':
            return 'Right'
        if str(w27.right_lane_marking.type) == 'Broken':
            return 'Left'
        return 'idk'

    
    def lane_change_eval(self,over_distance):
        
        t = self.vehicle.get_transform()
        v=self.vehicle.get_velocity()

        nwp = self.map.get_waypoint(t.location,project_to_road=True,lane_type=(carla.LaneType.Driving))

        #Preventing junction changes
        right_change_avaliable = ((nwp.lane_change==1 or nwp.lane_change ==3) and nwp.right_lane_marking.type == 1)
        left_change_avaliable = ((nwp.lane_change==2 or nwp.lane_change ==3) and nwp.left_lane_marking.type == 1)

        object_detected = False        
        
        for i in self.current_blocked:
            point =  self.blocked_points[i]
            dis_from_object_x = (t.location.x - point[0])+.5*v.x
            dis_from_object_y = (t.location.y - point[1])+.5*v.y

            xcat = 10 #Detection distance for after object passing
            if abs((180/3.14*math.atan2(v.y,v.x))%360-(180/3.14*math.atan2((point[1]-t.location.y),(point[0]-t.location.x)))%360)<30 and math.sqrt(dis_from_object_x**2+dis_from_object_y**2) < 40:
                xcat=40 #Detection distance for before object

            if math.sqrt(dis_from_object_x**2+dis_from_object_y**2) < xcat: #obstacle bounding box
            
                object_detected = True
                self.near_obj = i

                if self.new_object_feild:
                    # print("Object Detected, id: " + str(i))
                    # print(latdistance)
                    self.new_object_feild = False


                
        if not object_detected and not self.new_object_feild:
            self.new_object_feild = True   

        if object_detected:
            # print("Approaching, id: " + str(self.near_obj))
            point =  self.blocked_points[self.near_obj]
            dis_from_object_x = (t.location.x - point[0])+.5*v.x
            dis_from_object_y = (t.location.y - point[1])+.5*v.y
            if self.near_obj in self.unknown_obj:
                return 0
            
            if math.sqrt(dis_from_object_x**2+dis_from_object_y**2) < over_distance:
                # print("Over_distance fufilled")
                if left_change_avaliable and not right_change_avaliable:
                    return -2
                elif right_change_avaliable:
                    return 0
                elif self.laneinvade:
                    return -1
                else:
                    # print("Traffic Rules Broken")
                    return -2
            else:
                # print("Over_distance not fufilled",math.sqrt(dis_from_object_x**2+dis_from_object_y**2) , over_distance)
                return 0
        else:
            if right_change_avaliable: #Default position is right lane
                return 2
            elif self.laneinvade:
                return 2
            else:
                return 0
        
        print("Lange Change Logic Incomplete")



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

    def get_control(self,waypoints, speed, desired_speed, dt, obj,over_distance, cornering_mult=1):
        steer, alpha = self.pure_pursuit.get_control(waypoints, speed, obj,over_distance)
        
        if alpha is None:
            self.pid.set_point = desired_speed
        else:
            self.pid.set_point = max(desired_speed*(1/(1+cornering_mult*abs(alpha))),5)
        a = self.pid.get_control(speed,dt)
        return a, steer

