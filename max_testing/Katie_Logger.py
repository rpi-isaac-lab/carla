#!/usr/bin/env python
import faulthandler; faulthandler.enable()

import glob
import os
import sys

try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import csv
import argparse
import random
import numpy
import numpy as np
import time
import pygame
import math

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

from pure_pursuit import PurePursuit,PurePursuitPlusPID



def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class Waypoint_Finder(object):
	# This class finds waypoint ids and state data
	# for the driven vehicle
	def __init__(self,world,waypointroadids,waypointlaneids,waypointdistances):
		self.world = world
		self.map=world.get_map()
		self.simulation_time = 0
		self._server_clock = pygame.time.Clock()
		self.world.on_tick(self.on_tick)
		self.waypointroadids = self.waypointfileProcessorint(waypointroadids)
		self.waypointlaneids = self.waypointfileProcessorint(waypointlaneids)
		self.waypointdistances = self.waypointfileProcessorfloat(waypointdistances)
		self.vehicle_spawn=self.map.get_waypoint_xodr(self.waypointroadids[0],self.waypointlaneids[0],self.waypointdistances[0])
		self.timing=0
		self.lapcount=0

		self.pd_prev_time = time.time()
		self.pp = PurePursuitPlusPID() #Uncomment for PID
		self.waypointids=self.waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/waypointIDS.csv')
		self.waypointroadids=self.waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointRoadIDS.csv')
		self.waypointlaneids=self.waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointLaneIDs.csv')
		self.waypointdistances=self.waypointfileProcessorfloat('/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointDistances.csv')
		self.desired_speed = 20 # meters/second
		self.cornering_speed_mult = 5
	
		self.collision = None
		self.laneinvade = None
		pygame.joystick.init()
		self._joystick = pygame.joystick.Joystick(0)
		self._joystick.init()

		self._parser = ConfigParser()
		self._parser.read('wheel_config.ini')
		self._steer_idx = int(self._parser.get('G29 Racing Wheel', 'steering_wheel'))

		numAxes = self._joystick.get_numaxes()
		self.jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]

		

	def on_tick(self,timestamp):
		self._server_clock.tick()
		self.server_fps = self._server_clock.get_fps()
		self.frame = timestamp.frame
		self.simulation_time = timestamp.elapsed_seconds
		
	def timer(self,time):
		if time-self.timing>1:#Change to change the time between checks of being within the start zone
			self.timing=time
			
			return True
		else:
			return False
		
	def waypointfileProcessorfloat(self,csv_file):
		column_data = []
		with open(csv_file) as file:
			reader = csv.reader(file)
			next(reader, None)
			for row in reader:
				column_data.append(row)
			for i in range(len(column_data)):
				column_data[i] = float(column_data[i][0])
		return column_data
            
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
	
	def _on_collision(self,event):
		actor_type = get_actor_display_name(event.other_actor)
		self.collision = actor_type
		return actor_type
	
	def _on_invasion(self,event):
		lane_types = set(x.type for x in event.crossed_lane_markings)
		text = ['%r' % str(x).split()[-1] for x in lane_types]
		self.laneinvade = text

		return text
            
	def log_waypoints(self,filename,refresh_rate=60):# Refresh rate in Hz
		try:
			f = open(filename,"w+")
			f.write("time,Steering Angle,X Position,Y Position,Z Position, Pitch, Roll, Yaw, X velocity, Y velocity, Z velocity, Lateral Position,Lap Count, Lane Invasion, Collision Sensor\n")
			actor_list = self.world.get_actors()
			# Find the driver car
			for id in actor_list:
				try:
					if id.attributes["role_name"] == "hero":
						vehicle = id
					elif id.attributes["role_name"] == "collisionsensor":
						collisionsensor = id
					elif id.attributes["role_name"] == "lanesensor":
						lanesensor = id
					else:
						continue
				except:
					continue
			if vehicle is None and collisionsensor is None and lanesensor is None:
				return 1
			last_time = 0
			Collision=collisionsensor.listen(lambda event: self._on_collision(event))
			while True:
				curr_time = time.time()
				if curr_time-last_time >= 1/refresh_rate:
					last_time = curr_time
					t = vehicle.get_transform()
					v=vehicle.get_velocity()

					user_steer = self.joystick_control()
					auto_steer = self.pd_controller()

					nwp = self.map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving))
					wloc=nwp.transform #Get waypoint 3D location
					latdistance= math.sqrt((t.location.x-wloc.location.x)**2+(t.location.y-wloc.location.y)**2)
					if self.timer(curr_time):
						current_pos = vehicle.get_location()
						if current_pos.distance(self.vehicle_spawn.transform.location) < 10.0:
							self.lapcount += 1
							self.timing+=30
						Laneinvade=lanesensor.listen(lambda event: self._on_invasion(event))
					
					# # This recipe shows the current traffic rules affecting the vehicle. 
					# # Shows the current lane type and if a lane change can be done in the actual lane or the surrounding ones.

					# # ...
					waypoint27 = self.world.get_map().get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
					# if ((waypoint27.lane_change==1 or waypoint27.lane_change ==3) and waypoint27.right_lane_marking.type == 1):
					# 	print("Move Over")
					# 	print(waypoint27.lane_change,waypoint27.right_lane_marking.type)
					# # Left and Right lane markings

					

	
					f.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(self.simulation_time,vehicle.get_control().steer,t.location.x,t.location.y,t.location.z,t.rotation.pitch,t.rotation.roll,t.rotation.yaw,v.x,v.y,v.z,latdistance,self.lapcount,self.laneinvade,self.collision))
					# f.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(time.time(),vehicle.get_control().steer,t.location.x,t.location.y,t.location.z,t.rotation.pitch,t.rotation.roll,t.rotation.yaw,v.x,v.y,v.z,latdistance,self.lapcount,self.laneinvade,self.collision))
					if self.laneinvade != None:
						self.laneinvade = None
					if self.collision != None:
						self.collision = None
					
		except KeyboardInterrupt:
			f.close()
		return 0
	
	def pd_controller(self):
		
		v = self.world.player.get_velocity()
		map = self.world.world.get_map()
		speed_mps = (math.sqrt(v.x**2 + v.y**2 + v.z**2)) # Vehicle speed meters/sec
		current_time = time.time()
		delta_t = current_time-self.pd_prev_time
		waypoints = self.find_waypoints(self.world.player,map,inclusive=10)
		a, steer = self.pp.get_control(waypoints,speed_mps,self.desired_speed,delta_t,cornering_mult=self.cornering_speed_mult)
		# update the prev_time
		self.pd_prev_time = current_time
		
		return steer
	
	def joystick_control(self):
		K1 = 1.0  # 0.55
		deadzone = .05 # .035
		base_steering = self.jsInputs[self._steer_idx]
		if abs(base_steering) < deadzone:
			steerCmd = 440.4*base_steering**3
		else:
			steerCmd = K1 * math.tan(1.1 * base_steering)
		return steerCmd
	
	
	def find_waypoints(self,vehicle,map,number=200,max_dist=20,inclusive=None):
		# Find (number) waypoints from the vehicle forward along the map
		# If inclusive is not None, waypoints must be in list inclusive
		#This is currently creating a lot of lag, needs 200 waypoints to increase chances of finding one in the list already, but the searching is slow, could use improvement
		nwp = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving)) # Nearest waypoint to vehicle
		waypoints = [nwp]
		index=-1
		temp_var = False
		for i in range(number):
			wps = nwp.next(((i+1)/number)*max_dist)
			if len(wps) > 0:
				if wps[0].is_junction:
					if wps[0].junction_id==498: #Hardcoding a troublemaker
						for i in range(len(wps)):
							if wps[i].road_id==499 and wps[i].lane_id==1:
								waypoints.append(wps[i])
								break
							elif wps[i].road_id==551 and wps[i].lane_id==1:
								waypoints.append(wps[i])
								break
					elif wps[0].junction_id==861: #Hardcoding a troublemaker
						# print("flag")
						temp_var = True
						for i in range(len(wps)):
							if wps[i].road_id==874 and wps[i].lane_id==3:
								waypoints.append(wps[i])
								break
					elif wps[0].junction_id==238: #Hardcoding a troublemaker
						for i in range(len(wps)):
							if wps[i].road_id==271 and wps[i].lane_id==-1:
								waypoints.append(wps[i])
								break
					else:
						for i in range(len(wps)):
							if str(wps[i].lane_change)=='Left':
								waypoints.append(wps[i])
								break
				else:
					waypoints.append(wps[0])
		# Get vehicle matrix
		mat = np.array(vehicle.get_transform().get_inverse_matrix())
		waypoints = self.waypoints2locations(waypoints)
		#print("="*40)
		#print("="*40)
		#print(waypoints)
		#print("="*40)
		#print(mat)
		body_waypoints = np.zeros(shape=(len(waypoints),2))
		# Turn into 2d coords in car frame
		for i in range(len(waypoints)):
			waypoints[i] = mat@waypoints[i]
			temp = waypoints[i]/waypoints[i,3]
			body_waypoints[i] = temp[:2]
		#print("="*40)
		#print(waypoints)
		#print("="*40)
		#print("="*40)
		# if temp_var:
		#     print(body_waypoints)
		return body_waypoints
	
	def waypoints2locations(self,waypoints):
		locations = np.zeros(shape=(len(waypoints),4))
		for i in range(len(waypoints)):
			carla_loc = waypoints[i].transform.location
			loc_4 = [carla_loc.x,carla_loc.y,carla_loc.z,1] 
			locations[i,:] = np.array(loc_4)
		return locations



################################################################################
## TEST SCRIPTS ################################################################
def print_nearest_waypoint(world):
	map = world.get_map()
	actor_list = world.get_actors()
	for id in actor_list:
		#print(id)
		if id.attributes["role_name"] == "hero":
			vehicle = id
			break
	if vehicle is None:
		return
	nwp = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving))
	print("WayPointID {}: RoadID=({:.02f}), LaneID=({:.02f}), Sdistance({:.02f})".format(
		nwp.id,
		nwp.road_id,
		nwp.lane_id,
		nwp.s,
		))
	return

def follow_waypoints(client,filename):
	max_waypoints = 100
	world = client.get_world()
	map = world.get_map()
	waypoints = []
	wp = random.choice(map.generate_waypoints(10))
	for i in range(max_waypoints):
		wp = wp.next(.01)[0] # 1 meter increment waypoints
		pos = [wp.transform.location.x,wp.transform.location.y,10]#wp.transform.location.z]
		waypoints.append(pos)
	write2csv(filename,waypoints)


def write2csv(filename,data):
	with open(filename,"w+") as f:
		for item in data:
			output = ""
			for subitem in item:
				output += ",{}".format(subitem)
			output = output[1:] + "\n"
			f.write(output)
################################################################################


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
	filename = "KatieLogTest.csv"
	filename = "TheaLog100.csv"
	#follow_waypoints(client,filename)
	WP = Waypoint_Finder(client.get_world(),'/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv','/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv', '/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv')
	WP.log_waypoints(filename)
