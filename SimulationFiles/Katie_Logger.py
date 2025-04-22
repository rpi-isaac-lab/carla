#!/usr/bin/env python


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
import time
import pygame


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
            
	def log_waypoints(self,filename,refresh_rate=60):
		# Refresh rate in Hz
		try:
			f = open(filename,"w+")
			f.write("time,Steering Angle,X Position,Y Position,Z Position, Pitch, Roll, Yaw, X velocity, Y velocity, Z velocity, Lap Count\n")
			actor_list = self.world.get_actors()
			# Find the driver car
			for id in actor_list:
		            try:
		                if id.attributes["role_name"] == "hero":
		                    vehicle = id
		                    break
		            except:
		                continue
			if vehicle is None:
				return 1
			last_time = 0
			while True:
				curr_time = time.time()
				if curr_time-last_time >= 1/refresh_rate:
					last_time = curr_time
					t = vehicle.get_transform()
					v=vehicle.get_velocity()
					if self.timer(curr_time):
					    current_pos = vehicle.get_location()
					    if current_pos.distance(self.vehicle_spawn.transform.location) < 10.0:
					        self.lapcount += 1
					        self.timing+=30
					f.write("{},{},{},{},{},{},{},{},{},{},{},{},\n".format(self.simulation_time,vehicle.get_control().steer,t.location.x,t.location.y,t.location.z,t.rotation.pitch,t.rotation.roll,t.rotation.yaw,v.x,v.y,v.z,self.lapcount))
		except KeyboardInterrupt:
			f.close()
		return 0


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
	#follow_waypoints(client,filename)
	WP = Waypoint_Finder(client.get_world(),'/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv','/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv', '/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv')
	WP.log_waypoints(filename)
