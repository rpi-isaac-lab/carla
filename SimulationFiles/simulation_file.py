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

# spawns objects (cones) at 5 waypoints at a time, resetting the previous bjects and spawning new ones after each lap

def reset(world, waypointroadids, waypointlaneids, waypointdistances, object_type, start_index):
    actors = world.get_actors().filter(object_type)
    for actor in actors:
        actor.destroy()
    end_index = min(start_index + 5, len(waypointroadids))
    CONE(world, waypointroadids[start_index:end_index], waypointlaneids[start_index:end_index], waypointdistances[start_index:end_index], object_type) 

def simulation_file(world, vehicle, waypointroadids, waypointlaneids, waypointdistances, object_type, vehicle_spawn):
    lap_count = 0
    spawn_index = waypointroadids.index(vehicle_spawn.road_id)
    reset(world, waypointroadids, waypointlaneids, waypointdistances, object_type, spawn_index)
    
    while lap_count < 5:
        current_pos = vehicle.get_location()
        if current_pos.distance(vehicle_spawn.transform.location) < 5.0:
            lap_count += 1
            spawn_index = (spawn_index + 5) % len(waypointroadids)
            reset(world, waypointroadids, waypointlaneids, waypointdistances, object_type, spawn_index)
        time.sleep(1)

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
    vehicle = world.get_actors().find('vehicle.*')

    vehicle_spawn = map.get_waypoint(vehicle.get_location(), project_to_road=True, lane_type=(carla.LaneType.Driving))

    waypointroadids = waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv')
    waypointlaneids = waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv')
    waypointdistances = waypointfileProcessorfloat('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv')

    simulation_file(world, vehicle, waypointroadids, waypointlaneids, waypointdistances, "static.prop.trafficcone01", vehicle_spawn)