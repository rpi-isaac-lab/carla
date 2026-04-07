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
from cone3_copy import CONE1

def CONE(world):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """

    waypointroadids=np.loadtxt('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv',int,skiprows=1)
    waypointlaneids=np.loadtxt('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv',int,skiprows=1)
    waypointdistances=np.loadtxt('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv',float,skiprows=1)
    
    

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
        f = open("TheaLogger.csv","w+")
        f.close()

        self.waypointroadids = self.waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointRoadIDS.csv')
        self.waypointlaneids = self.waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointLaneIDs.csv')
        self.waypointdistances = self.waypointfileProcessorfloat('/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointDistances.csv')
        self.lapcount=0
        self.map = self.world.get_map()
        self.vehicle_spawn=self.map.get_waypoint_xodr(self.waypointroadids[0],self.waypointlaneids[0],self.waypointdistances[0])
        self.current_blocked = [100,201,1003,200]
        self.spawn_index = -5  
        self.simtime = time.time()
        self.new_object_feild = True
        self.lane_change_direction = -1
        #self.reset(object_type, spawn_index)     
        
        
    def update_lane_change(self,lane_change_direction = -1):
        self.lane_change = not self.lane_change
        self.lane_change_direction = lane_change_direction
     

        
        self.road_id = None
        self.lane_id = None

        print(self.lane_change)
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
        
    def waypointfileProcessorString(self,csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader,None)
            for row in reader:
                column_data.append(row[0].strip())
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
        if current_pos.distance(self.vehicle_spawn.transform.location) < 10.0 and time.time()-self.simtime > 5:
            self.lapcount += 1
            # print(self.lapcount)
            # print(self.blocked_points_index[int((self.lapcount-1)%4)])
            self.current_blocked = self.blocked_points_index[int((self.lapcount-1)%4)]
            self.simtime=time.time()#Change to change the time it roughly takes to get out of the start zone
        
        self.start_time = time.time()
    
    def _on_invasion(self,event):
        # print(event.crossed_lane_markings)
        self.laneinvade = True
        return
    
    def get_control(self, waypoints, speed,obj):
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
            self.update_lapcount()
        except:
            cat = 0 
            
        if obj:
            self.lane_change_eval(waypoints)
            # self.lane_shift = self.lane_change_eval(waypoints)
        
        try:
            if self.lane_change:
                t = self.vehicle.get_transform()
                nwp = self.map.get_waypoint(self.vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving))
                wloc=nwp.transform #Get waypoint 3D location
                latdistance= math.sqrt((t.location.x-wloc.location.x)**2+(t.location.y-wloc.location.y)**2)
                waypoint27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
                # print(waypoint27.transform.location)
                print(self.lane_id, waypoint27.lane_id)
                if self.lane_id == None:
                    self.lane_id = waypoint27.lane_id
                    # print("Lane id set: ", self.lane_id)
                if self.lane_id == waypoint27.lane_id:
                    # print(waypoint27.lane_width)
                    # print(latdistance)
                    self.lane_shift = self.lane_change_direction *2
                else:
                    # print(waypoint27.lane_id)
                    t = self.vehicle.get_transform()

                    nwp = self.map.get_waypoint(self.vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving))
                    wloc=nwp.transform #Get waypoint 3D location
                    latdistance= math.sqrt((t.location.x-wloc.location.x)**2+(t.location.y-wloc.location.y)**2)
                    
                    if latdistance < 1:
                        self.update_lane_change()
                        self.road_id = None
                        self.lane_id = None
                    else:
                        self.lane_shift=0
        except Exception as e:
            print(e)
        # print(self.lane_shift)
        waypoints[:,1]+=self.lane_shift

        

        current_vehicle_location = self.vehicle.get_transform()
        
                    

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
        track_point = get_target_point(look_ahead_distance, waypoints)
        
        if track_point is None:
            return None, None

        # print(track_point)
        t = self.vehicle.get_transform()
        # nwp1 = nwp.transform
        # nwp1.location.x+=track_point[0]*v.x/speed
        # nwp1.location.y+=track_point[1]*v.y/speed
        CONE1(self.world,"static.prop.gnome",t)
        alpha = np.arctan2(track_point[1], track_point[0])
        # print(alpha)
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
        # if nwp.road_id == 1959:
        #     if nwp.s < 20:
        #         steer = steer/2
        #     print(nwp.s,steer*(180/3.14))
        t = self.vehicle.get_transform()
        # print(t.location.x, t.location.y)
        # print()

        # if t.location.x > -60:
        #     print(nwp.road_id, steer)
        
        if nwp.road_id == 1933 and steer < 0:
            return -steer, alpha
        if steer > .4:
            waypoint27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
            # print(waypoint27.road_id)
            
            if waypoint27.is_junction:
                # print(waypoint27.junction_id)
                # if waypoint27.junction_id == 861:
                

                # print("Override")
                
                return 0,0
        return steer, alpha
    
    def lane_change_eval(self,waypoints):
        waypoint27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))

        t = self.vehicle.get_transform()
        v=self.vehicle.get_velocity()

        nwp = self.map.get_waypoint(self.vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving))
        wloc=nwp.transform #Get waypoint 3D location
        latdistance= math.sqrt((t.location.x-wloc.location.x)**2+(t.location.y-wloc.location.y)**2)

        # if self.lane_shift == 0:
        #     for point in self.blocked_points:
        #         if abs(t.location.x - point[0]) < 10 and abs(t.location.y - point[1]) < 10:
        #             print("WARNING OBJECT")
        #             if ((waypoint27.lane_change==2 or waypoint27.lane_change ==3) and waypoint27.left_lane_marking.type == 1):
        #                 self.lane_shift = -2
        #             elif not self.laneinvade:
        #                 self.lane_shift = 0

        right_change_avaliable = ((waypoint27.lane_change==1 or waypoint27.lane_change ==3) and waypoint27.right_lane_marking.type == 1)
        left_change_avaliable = ((waypoint27.lane_change==2 or waypoint27.lane_change ==3) and waypoint27.left_lane_marking.type == 1)

        object_detected = False
        
        for i, point in enumerate(self.blocked_points):
            dis_from_object_x = (t.location.x - point[0])+.5*v.x
            dis_from_object_y = (t.location.y - point[1])+.5*v.y
            if -10 < dis_from_object_x < 10 and -10 < dis_from_object_y < 10: #obstacle bounding box
                # print(i, self.current_blocked, i in self.current_blocked)
                
                if i in self.current_blocked:
                    
                    # print(dis_from_object_x, dis_from_object_y)
                # if True:
                    # print(i)
                    # if time.time()-self.start_time > 20:
                    object_detected = True
                    if self.new_object_feild:
                        # print("Object Detected, id: " + str(i))
                        # print(latdistance)
                        self.new_object_feild = False
                    if i in [1,3,8, 9,10,12,13,15,18,23,26]:
                        
                        
                        self.track_point = get_target_point(self.look_ahead_distance, waypoints)
                        # print(time.time())
                        
                    # print(dis_from_object_x/v.x, dis_from_object_y/v.y)
                        print(i)

                # print(object_detected)  
                # 
        if not object_detected and not self.new_object_feild:
            self.new_object_feild = True   
                
        # f_r = open("TheaLogger.csv", "r")
        # f_read=f_r.read()
        # f_r.close()
        # f = open("TheaLogger.csv","w+")
        # f.write(f_read)
        # # f.write("{},{},{},{},{},{},{},{},{},{}\n".format(time.time(),i,t.location.x,point[0],dis_from_object_x,v.x,t.location.y,point[1],dis_from_object_y,v.y))
        # # for point in waypoints:
        # #     f.write("{},{}\n".format(point[0]+t.location.x,point[1]+t.location.y))
        # f.close()

        
        waypoint27 = self.world.get_map().get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
        

        if object_detected:
            if left_change_avaliable and not right_change_avaliable:
                # print(-2/1.5*latdistance, )
                # return -.5-latdistance
                return -2
            elif right_change_avaliable:
                return 0
            elif self.laneinvade:
                return -1
            else:
                # print("Traffic Rules Broken")
                return -2
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

    def get_control(self,waypoints, speed, desired_speed, dt, obj, cornering_mult=1):
        steer, alpha = self.pure_pursuit.get_control(waypoints, speed, obj)
        #print(alpha)
        if alpha is None:
            self.pid.set_point = desired_speed
        else:
            self.pid.set_point = max(desired_speed*(1/(1+cornering_mult*abs(alpha))),5)
        a = self.pid.get_control(speed,dt)
        return a, steer


class Lapping():
    def __init__(self,waypointroadids,waypointlaneids,waypointdistances,vehicle):
        """A function to intialize an Obstacles spawing object
        
        Given: 
        waypointroadids: The string of the location of the csv file with waypoint roadids
        waypointlaneids: The string of the location of the csv file with waypoint laneids
        waypointdistances: The string of the location of the csv file with waypoint distances
        worldobject: The world as the object class defined in this file
        worldcarla: The carla world object
        """
        self.waypointroadids = self.waypointfileProcessorint(waypointroadids)
        self.waypointlaneids = self.waypointfileProcessorint(waypointlaneids)
        self.waypointdistances = self.waypointfileProcessorfloat(waypointdistances)
        self.lapcount=0
        self.vehicle=vehicle
        self.vehicle_spawn=self.map.get_waypoint_xodr(self.agent.waypointroadids[0],self.agent.waypointlaneids[0],self.agent.waypointdistances[0])
        self.spawn_index = -5  
        self.simtime = time.time()
        #self.reset(object_type, spawn_index)     
        
        
    
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
        
    def waypointfileProcessorString(self,csv_file):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader,None)
            for row in reader:
                column_data.append(row[0].strip())
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
        if current_pos.distance(self.vehicle_spawn.transform.location) < 10.0 and time.time()-self.simtime > 5:
            self.lapcount += 1
            
            self.simtime=time.time()#Change to change the time it roughly takes to get out of the start zone

            
                
    
  