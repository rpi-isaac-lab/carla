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


def spawn_kid(world, n, waypoint_location):
    # Get the blueprint library from the world
    library = world.get_blueprint_library()
    # Use the identifier from the blueprint library to search for the blueprint
    kid = library.find("walker.pedestrian.0004")

    if waypoint_location:
        waypoint = map.get_waypoint(carla.Location(*waypoint_location))
    else:
        waypoint = None

    for i in range(n):
        if waypoint:
            spawn = waypoint.transform
        else:
            spawn = random.choice(map.get_waypoint())

        this_kid = world.spawn_actor(kid,spawn)
        this_kid.set_enable_gravity(True)
        this_kid.set_simulate_physics(True)
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

    spawn_kid(world, 10, waypoint_location)
