#!/usr/bin/env python

"""

 ####  ###  #   # #####
#     #  ## ##  # #    
#     # # # # # # ###  
#     ##  # #  ## #    
 ####  ###  #   # ##### 

"""

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

def CONE(world, waypointroadids, waypointlaneids, waypointdistances, obstacletype):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """

    map = world.get_map()
    cone_library = world.get_blueprint_library()
    cone = cone_library.find(obstacletype)

    for i in range(len(waypointroadids)):
        waypoint = map.get_waypoint_xodr(waypointroadids[i], waypointlaneids[i], waypointdistances[i])
        if waypoint:
            spawn = waypoint.transform
            this_cone = world.spawn_actor(cone,spawn)
            this_cone.set_enable_gravity(True)
            this_cone.set_simulate_physics(True)
    return

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

    waypointroadids = waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv')
    waypointlaneids = waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv')
    waypointdistances = waypointfileProcessorfloat('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv')
    
    CONE(world, waypointroadids, waypointlaneids, waypointdistances, "static.prop.trafficcone01")
