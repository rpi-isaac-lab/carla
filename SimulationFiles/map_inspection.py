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


def explore_map_wp(world):
	map = world.get_map()
	waypoint_list = map.get_topology()
	for i in range(len(waypoint_list)):
		print("Start:\n{}\nEnd:\n{}\n".format(wp2str(waypoint_list[i][0]),
																					wp2str(waypoint_list[i][1])))
	return


def wp2str(waypoint):
	return "Road_ID: {}\nSection_ID: {}\nLane_ID: {}\nID: {}\nTransform:[{:.2f},{:.2f},{:.2f}]".format(waypoint.road_id,
												waypoint.section_id,
												waypoint.lane_id,
												waypoint.id,
												waypoint.transform.location.x,
												waypoint.transform.location.y,
												waypoint.transform.location.z)


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
	explore_map_wp(world)
	