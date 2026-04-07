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
import time

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
import numpy as np


def CONE(world, obstacletype, loop_count = 1):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """
    waypointroadids=np.loadtxt('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsRoadIDs.csv',int,skiprows=1)
    waypointlaneids=np.loadtxt('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsLaneIDs.csv',int,skiprows=1)
    waypointdistances=np.loadtxt('/home/labstudent/carla/PythonAPI/max_testing/Data/ExactObjectWaypointsS.csv',float,skiprows=1)
    blocked_points_index = [[4, 3, 17, 11], [2, 19, 0, 5], [24, 22, 14, 6], [7, 16, 25, 21]]
    blocked_points = blocked_points_index[int((loop_count-1)%4)]
    print("Loop" , int((loop_count-1)%4))
    # print(blocked_points)
    map = world.get_map()

    actor_list = world.get_actors()
    for id in actor_list:
        try:
            if id.attributes["role_name"] == "cone_bob":
                id.set_enable_gravity(False)
                cone_loc = id.get_location()
                cone_loc.z += 100
                id.set_location(cone_loc)
        except:
            continue

    cone_library = world.get_blueprint_library()
    cone = cone_library.find(obstacletype)
    cone.set_attribute('role_name', 'cone_bob')


    for i in range(len(waypointroadids)):
        waypoint = map.get_waypoint_xodr(int(waypointroadids[i]), int(waypointlaneids[i]), float(waypointdistances[i]))
        if waypoint:
            if i in blocked_points:
                spawn = waypoint.transform
                # print(spawn)
                this_cone = world.spawn_actor(cone,spawn)
                this_cone.set_enable_gravity(True) #Turn this back on
                this_cone.set_simulate_physics(True)
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
    
    t0 = time.time()

    while 5 > (time.time() - t0):
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(0.1)
            #print(client.get_server_version())
            print('CARLA %s connected at %s:%d.' % (client.get_server_version(), args.host, args.port))
            break

        except RuntimeError:
            pass
            


    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    
    
    # CONE(world,"static.prop.trafficcone01")
    
