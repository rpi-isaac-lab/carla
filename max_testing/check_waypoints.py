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
	print("WayPoint {}: Loc=({:.02f},{:.02f},{:.02f}), Rot(PRY)=({:.02f},{:.02f},{:.02f})".format(
		nwp.id,
		nwp.transform.location.x,
		nwp.transform.location.y,
		nwp.transform.location.z,
		nwp.transform.rotation.pitch,
		nwp.transform.rotation.roll,
		nwp.transform.rotation.yaw))
	return





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
	while True:
		world = client.get_world()
		print_nearest_waypoint(world)
		time.sleep(0.5)
