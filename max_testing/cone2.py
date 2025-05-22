#!/usr/bin/env python

"""

#   #  ###  #####   ####  ###  #   # #####
##  # #  ##   #    #     #  ## ##  # #    
# # # # # #   #    #     # # # # # # ###  
#  ## ##  #   #    #     ##  # #  ## #    
#   #  ###    #     ####  ###  #   # ##### 

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


def PROP(world, waypointroadids,waypointlaneids,waypointdistances,obstacletype):
    """
        Plops specified obstacles at specified points through an array
    """

    map = world.get_map()
    prop_library = world.get_blueprint_library()
    
    j=0
    for i in range(len(waypointroadids)):
        prop = prop_library.find(obstacletype[j])
        waypoint = map.get_waypoint_xodr(waypointroadids[i], waypointlaneids[i], waypointdistances[i])
        if waypoint:
            spawn = waypoint.transform
            this_prop = world.spawn_actor(prop,spawn)
            this_prop.set_enable_gravity(True)
            this_prop.set_simulate_physics(True)
        j+=1
        if j==4:
            j=0
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

    waypointroadids=waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv')
    waypointlaneids=waypointfileProcessorint('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv')
    waypointdistances=waypointfileProcessorfloat('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv')
    
    obstacletype=["static.prop.trafficcone01","static.prop.foodcart","static.prop.glasscontainer","static.prop.bin"]
    
    PROP(world,waypointroadids,waypointlaneids,waypointdistances,obstacletype)
    
