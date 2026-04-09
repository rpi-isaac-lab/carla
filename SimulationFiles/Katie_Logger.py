#!/usr/bin/env python
import faulthandler; faulthandler.enable()

import glob
import os
import sys

try:
    this_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(glob.glob('%s/../carla/dist/carla-*%d.%d-%s.egg' % (
        this_dir,
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
from cone3_copy import CONE1


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class Waypoint_Finder(object):
	# This class finds waypoint ids and state data
	# for the driven vehicle
	def __init__(self,world,waypointroadids,waypointlaneids,waypointdistances,args):
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
		self.desired_speed = 20 # meters/second
		self.cornering_speed_mult = 5
	
		self.collision = None
		self.laneinvade = None
		pygame.init()
		pygame.joystick.init()
		self._joystick = pygame.joystick.Joystick(0)
		self._joystick.init()

		self._parser = ConfigParser()
		self._parser.read('wheel_config.ini')
		self._steer_idx = int(self._parser.get('G29 Racing Wheel', 'steering_wheel'))
		self._throttle_idx = int(
			self._parser.get('G29 Racing Wheel', 'throttle'))
		self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
		
		numAxes = self._joystick.get_numaxes()
		self.jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
		self.base_steer = self.jsInputs[self._steer_idx]

		self.obj = args.obstacles

		self.left_turn_signal = False
		self.right_turn_signal = False

		self.score = 0
		self.cooldowntim = time.time()
		self.parking = False
		


		

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
		if time.time()-self.cooldowntim > 5:
			# print("Ow")
			self.cooldowntim = time.time()
			self.score -= 10
		return actor_type
	
	def _on_invasion(self,event):
		lane_types = set(x.type for x in event.crossed_lane_markings)
		text = ['%r' % str(x).split()[-1] for x in lane_types]
		self.laneinvade = text

		return text
            
	def log_waypoints(self,filename,refresh_rate=60):# Refresh rate in Hz
		try:
			f = open(filename,"w+")
			f.write("time,Lap Count, ")
			f.write("Lateral Position,Score,Lane Invasion, Collision Sensor,Lane Change,")
			f.write("Steering Angle,User Steering Angle,Controler Steering Angle,")
			f.write("Throttle,User Throttle,User Brake,Controller Accel,")
			f.write("User X Position,User Y Position,User Z Position, User Pitch, User Roll, User Yaw,")
			f.write("User X velocity, User Y velocity, User Z velocity,")
			f.write("wloc X Position,wloc Y Position,wloc Z Position, wloc Pitch, wRoll, wloc Yaw,") 
			f.write("X Center Ahead, Y Center Ahead, Z Center Ahead,")
			f.write("X Shoulder Ahead, Y Shoulder Ahead, Z Shoulder Ahead\n")
			
			# Find the driver car
			vehicle = None
			collisionsensor = None
			lanesensor = None
			while vehicle == None or collisionsensor == None or lanesensor== None:
				actor_list = self.world.get_actors()
				for id in actor_list:
					try:
						if id.attributes["role_name"] == "hero":
							vehicle = id
							self.vehicle = id
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
				# while not done:
				# 	for event in pygame.event.get():
				# 		if event.type == pygame.QUIT:
				# 				done = True  # Flag that we are done so we exit this loop.
				# print(pygame.event.get())
				for event in pygame.event.get():
					# print(event.type)
					if event.type == 1536:
						self.base_steer = event.value
					if event.type == pygame.JOYBUTTONDOWN:
						if event.button == 5:
							
							self.left_turn_signal = True
							# self.turnsignalontime = time.time()
							# if self.hud.turn ==0:
							# 		self.lanegoal = 'Right'
							# 		self.hud.turn = -1
							# 		self._agent.pp.pure_pursuit.update_lane_change(-1)
							# else:
							# 		self.hud.turn = 0       
							# 		self._agent.pp.pure_pursuit.update_lane_change(0)               
						elif event.button == 4:
							
							
							self.right_turn_signal = True
							# self.turnsignalontime = time.time()

							# if self.hud.turn ==0:
							# 		self.lanegoal = 'Left'
							# 		self.hud.turn = 1
							# 		self._agent.pp.pure_pursuit.update_lane_change(1)
							# else:
							# 		self.hud.turn = 0
							# 		self._agent.pp.pure_pursuit.update_lane_change(0)
						if event.button == 0:
							self.parking = not self.parking
				curr_time = time.time()
				if curr_time-last_time >= 1/refresh_rate:

					last_time = curr_time
					t = vehicle.get_transform()
					v = vehicle.get_velocity()

					user_steer, user_throttle, user_brake = self.joystick_control()
					a, auto_steer = self.pd_controller()
					# print(user_steer)

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
					
					try:
						waypoint27 = self.world.get_map().get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
						center_ahead = waypoint27.next(6.3)[0].transform.location
						center_ahead_x = center_ahead.x
						center_ahead_y = center_ahead.y
						center_ahead_z = center_ahead.z
						
						
					except:
						center_ahead_x = None
						center_ahead_y = None
						center_ahead_z = None
					try:
						waypoint27 = self.world.get_map().get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Shoulder))
						shoulder_ahead = waypoint27.next(20)[0].transform.location
						shoulder_ahead_x = shoulder_ahead.x
						shoulder_ahead_y = shoulder_ahead.y
						shoulder_ahead_z = shoulder_ahead.z
						# CONE1(self.world,"static.prop.gnome",waypoint27.next(20)[0].transform)
					except:
						shoulder_ahead_x = None
						shoulder_ahead_y = None
						shoulder_ahead_z = None

					try: #changed from world.get_location to vehicle.get_location and added a definition for speed_kmh which was not present here.
						w27 = self.world.get_map().get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
						self.speed_kmh = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
						if str(w27.right_lane_marking.type) == 'Solid':
							self.score+=1/200.*self.speed_kmh/(1.6*40.)
						elif w27.is_junction:
							self.score+=1/200.*self.speed_kmh/(1.6*40.)
								# self.lane = 'Junction'

						else:
							self.score+=1/200.*self.speed_kmh/(1.6*40.)

					except:
						self.score *= 1.
					
					lane_change = 0
					if self.left_turn_signal:
						lane_change -= 1
					if self.right_turn_signal:
						lane_change += 1

	
					

					f.write("{},{},".format(self.simulation_time,self.lapcount))
					f.write("{},{},{},{},{},".format(latdistance,self.score,self.laneinvade,self.collision,lane_change))
					f.write("{},{},{},".format(vehicle.get_control().steer,user_steer,auto_steer))
					f.write("{},{},{},{},".format(vehicle.get_control().throttle,user_throttle,user_brake,a))
					f.write("{},{},{},{},{},{},".format(t.location.x,t.location.y,t.location.z,t.rotation.pitch,t.rotation.roll,t.rotation.yaw))
					f.write("{},{},{},".format(v.x,v.y,v.z))
					f.write("{},{},{},{},{},{},".format(wloc.location.x,wloc.location.y,wloc.location.z,wloc.rotation.pitch,wloc.rotation.roll,wloc.rotation.yaw)) 
					f.write("{},{},{},".format(center_ahead_x,center_ahead_y,center_ahead_z))
					f.write("{},{},{}\n".format(shoulder_ahead_x,shoulder_ahead_y,shoulder_ahead_z))
					
					if self.laneinvade != None:
						self.laneinvade = None
					if self.collision != None:
						self.collision = None
					if self.right_turn_signal:
						self.right_turn_signal = False
					if self.left_turn_signal:
						self.left_turn_signal = False
					
		except KeyboardInterrupt:
			f.close()
		return 0
	
	def pd_controller(self):
		
		v = self.vehicle.get_velocity()
		map = self.world.get_map()
		speed_mps = (math.sqrt(v.x**2 + v.y**2 + v.z**2)) # Vehicle speed meters/sec
		current_time = time.time()
		delta_t = current_time-self.pd_prev_time
		waypoints = self.find_waypoints(self.vehicle,map,inclusive=10)
		a, steer = self.pp.get_control(waypoints,speed_mps,self.desired_speed,delta_t,self.obj,20,cornering_mult=self.cornering_speed_mult)
		# update the prev_time
		self.pd_prev_time = current_time
		
		return a, steer
	
	def joystick_control(self):
		K1 = 1.0  # 0.55
		deadzone = .05 # .035
		
		numAxes = self._joystick.get_numaxes()
		jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
		base_steering = jsInputs[self._steer_idx]
		base_steering = self.base_steer
		if abs(base_steering) < deadzone:
			steerCmd = 440.4*base_steering**3
		else:
			steerCmd = K1 * math.tan(1.1 * base_steering)

		K2 = 1.6  # 1.6
		throttleCmd = K2 + (2.05 * math.log10(-0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
		if throttleCmd <= 0:
			throttleCmd = 0
		elif throttleCmd > 1:
			throttleCmd = 1

		brakeCmd = 1.6 + (2.05 * math.log10(-0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
		if brakeCmd <= 0:
			brakeCmd = 0
		elif brakeCmd > 1:
			brakeCmd = 1
		
		return steerCmd, throttleCmd, brakeCmd
	
	
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
		body_waypoints = np.zeros(shape=(len(waypoints),2))
		# Turn into 2d coords in car frame
		for i in range(len(waypoints)):
			waypoints[i] = mat@waypoints[i]
			temp = waypoints[i]/waypoints[i,3]
			body_waypoints[i] = temp[:2]
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
		)) #active print
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
	argparser.add_argument(
		'-t', '--trial',
		default=0,
		type=int,
		help='Trial Type')
	argparser.add_argument(
		'-d', '--id',
		default=0,
		type=int,
		help='Participent ID')
	argparser.add_argument(
		'-o', '--obstacles',
		default=False,
		type=bool,
		help='Obstacle presence (default on)')
	argparser.add_argument(
		'-op', '--obstacles_present',
		default=False,
		type=bool,
		help='Obstacle presence (default on)')
	args = argparser.parse_args()
	

  
	client = carla.Client(args.host, args.port)
	client.set_timeout(2.0)
	
	#Input Participant Number in terminal
	# name = int(input("Enter your participant number:"))
	
	
	#load data array of all participant numbers and assosciated run counts
	#Currently only one run counter, different types could be different arrays or different enteries
	
	i = 1
	while True:
		filename = "ParticipantData/"+"T"+str(args.trial)+"/"+"P"+str(args.id)+"R" + str(i)+"Log.csv"
		try:
			f = open(filename,"r")
			f.close()
			i += 1
		except:
			break

	

	#Filename in format P{participant number}R{run_number}Log.csv
	filename = "/home/labstudent/carla/ParticipantData/"+"T"+str(args.trial)+"/"+"P"+str(args.id)+"R" + str(i)+"Log.csv"

	WP = Waypoint_Finder(client.get_world(),'/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointRoadIDS.csv','/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointLaneIDs.csv', '/home/labstudent/carla/PythonAPI/max_testing/Data/WaypointDistances.csv',args)
	WP.log_waypoints(filename)
