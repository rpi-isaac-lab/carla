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

import argparse
import random


def CONE(world, n, waypoint_location):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """

    map = world.get_map()
    cone_library = world.get_blueprint_library()
    cone = cone_library.find("static.prop.trafficcone01")

    # if the waypoint location is provided, find the nearest waypoint
    if waypoint_location:
        waypoint = map.get_waypoint(carla.Location(*waypoint_location))
    else:
        waypoint = None

    for i in range(n):
        # if the waypoint location is provided, use this location to spawn the cone
        if waypoint:
            spawn = waypoint.transform
        else: 
            spawn = random.choice(map.get_waypoint())

        this_cone = world.spawn_actor(cone,spawn)
        this_cone.set_enable_gravity(True)
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
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    if args.x is not None and args.y is not None and args.z is not None:
        waypoint_location = (args.x, args.y, args.z)
    else:
        waypoint_location = None

    CONE(world, 1000, waypoint_location)
