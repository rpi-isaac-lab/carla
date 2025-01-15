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


def cone_formation(world,coords):
    cone_library = world.get_blueprint_library()
    cone = cone_library.find("static.prop.trafficcone01")
    for coord in coords:
        spawn = carla.Transform(carla.Location(x=coord[0],y=coord[1],z=coord[2]))
        this_cone = world.spawn_actor(cone,spawn)
        this_cone.set_enable_gravity(True)
        this_cone.set_simulate_physics(True)
    return


ht = 2.5
TOWN03_COORDS = [
  [233.8165,-14.40,ht],
  [233.8165,-18.69,ht],
	[233.8165,-22.00,ht],
  [233.8165,-26.00,ht],
	[233.0000,-30.00,ht],
  [232.1560,-34.00,ht],
	[231.3500,-38.00,ht]
]


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
    cone_formation(world,TOWN03_COORDS)
