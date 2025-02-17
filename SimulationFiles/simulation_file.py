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

import time

from cone import CONE

def preload_simulation(world, spawn_instructions):
    waypoints = []
    start_time = time.time()

    # sort the spawn instructions by time
    spawn_instructions.sort(key=lambda x: x[0])

    for instruction in spawn_instructions:
        spawn_time, _, road_id, lane_id, s = instruction
        elapsed_time = time.time() - start_time
        remaining_time = spawn_time - elapsed_time

        if remaining_time > 0:
            time.sleep(remaining_time)
            
        waypoint = world.get_map().get_waypoint_xodr(road_id, lane_id, s)
        if waypoint:
            waypoints.append(waypoint)
            
    CONE(world, waypoints)

def waypointfileProcessorint(self, csv_file):
    column_data = []
    with open(csv_file) as file:
        reader = csv.reader(file)
        next(reader, None)
        for row in reader:
            column_data.append(row)
        for i in range(len(column_data)):
            column_data[i] = int(column_data[i][0])
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

    spawn_instructions = waypointfileProcessorint('ExactObjectWaypoints.csv')
    preload_simulation(world, spawn_instructions)