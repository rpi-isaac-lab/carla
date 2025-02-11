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
import math

from cone import CONE

passed_count = {}

def get_dist(vehicle_loc, specific_point):
    return math.sqrt((vehicle_loc.x - specific_point[0])**2 + (vehicle_loc.y - specific_point[1])**2 + (vehicle_loc.z - specific_point[2])**2)

def track_vehicle(world, vehicle, specific_point, waypoint):
    global passed_count
    vehicle_id = vehicle.id
    while True:
        transform = vehicle.get_transform()
        vehicle_loc = transform.location
        if get_dist(vehicle_loc, specific_point) < 5.0: # within 5 meters
            if vehicle_id not in passed_count:
                passed_count[vehicle_id] = 0
            passed_count[vehicle_id] += 1
            if passed_count[vehicle_id] == 2:
                CONE(world, waypoint)

def parse_file(filename, world, vehicle, n):
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue # skip lines that don't have enough data
            _, _, road_id, lane_id, s = parts # ignore time and id
            road_id, lane_id, s = int(road_id), int(lane_id), float(s)
            waypoints.append((road_id, lane_id, s))
    
    # point to keep track of when counting laps
    specific_point = waypoints[0]
    # waypoint to spawn the cone
    waypoint = random.sample(waypoints, n)
    track_vehicle(world, vehicle, specific_point, waypoint)

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

    vehicle = world.get_actors().filter('vehicle*')[0]

    parse_file('recreatewaypoint.csv', world, vehicle, 1)