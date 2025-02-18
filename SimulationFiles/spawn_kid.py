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


def spawn_kid(world, waypoints):
    map = world.get_map()
    # Get the blueprint library from the world
    library = world.get_blueprint_library()
    # Use the identifier from the blueprint library to search for the blueprint
    kid = library.find("walker.pedestrian.0004")

    for road_id, lane_id, s in waypoints:
        waypoint = map.get_waypoint_xodr(road_id, lane_id, s)
        if waypoint:
            spawn = waypoint.transform
            this_kid = world.spawn_actor(kid,spawn)
            this_kid.set_enable_gravity(True)
            this_kid.set_simulate_physics(True)
    return

def waypointfileProcessorint(self, csv_file, world, n):
        column_data = []
        with open(csv_file) as file:
            reader = csv.reader(file)
            next(reader, None)
            for row in reader:
                column_data.append(row)
            for i in range(len(column_data)):
                 column_data[i] = int(column_data[i][0])
        waypoints_used = random.sample(column_data, n)
        spawn_kid(world, waypoints_used)

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

    waypointfileProcessorint("waypoints.csv", world, 10)
