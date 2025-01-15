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

import argparse
import random
import time

def find_nearest_waypoint(world):
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
	return nwp


def follow_cone(client,name,distance):
	world = client.get_world()
	cone_library = world.get_blueprint_library()
	cone = cone_library.find(name)
	spawn = random.choice(world.get_map().get_spawn_points())
	this_cone = world.spawn_actor(cone,spawn)
	this_cone.set_enable_gravity(False)
	this_cone.set_simulate_physics(False)
	while True:
		try:
			world = client.get_world()
			nwp = find_nearest_waypoint(world)
			nwp_ahead = nwp.next(distance)[0]
			this_cone.set_transform(nwp_ahead.transform)
		except KeyboardInterrupt:
			this_cone.destroy()
			return



def write2csv(filename,data):
	with open(filename,"w+") as f:
		for item in data:
			output = ""
			for subitem in item:
				output += ",{}".format(subitem)
			output = output[1:] + "\n"
			f.write(output)			


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
	cone = "static.prop.trafficcone01"
	car = "vehicle.lincoln.mkz_2020"
	follow_cone(client,car,15)
