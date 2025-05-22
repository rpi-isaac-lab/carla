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


def CONE(world,n):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.
    """
    cone_library = world.get_blueprint_library()
    smol_cone = cone_library.find("static.prop.constructioncone")
    garbaaaage = cone_library.find("static.prop.container")
    cone = cone_library.find("static.prop.trafficcone01")
    barb = cone_library.find("static.prop.barbeque")
    for i in range(n):
        spawn = random.choice(world.get_map().get_spawn_points())
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
    CONE(world,1000)
