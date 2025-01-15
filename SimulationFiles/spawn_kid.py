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


def spawn_kid(world,n):
    # Get the blueprint library from the world
    library = world.get_blueprint_library()
    # Use the identifier from the blueprint library to search for the blueprint
    kid = library.find("walker.pedestrian.0004")
    for i in range(n):
        # Find a random spawn from the map
        spawn = random.choice(world.get_map().get_spawn_points())
        # Instantiate the object
        this_kid = world.spawn_actor(kid,spawn)
        # Gravity
        this_kid.set_enable_gravity(True)
        # Decides whether or not it will go flying
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
    spawn_kid(world,10)
