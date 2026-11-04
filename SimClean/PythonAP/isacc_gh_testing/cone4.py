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


def spawn_CONE(world, obstacletype, obstaclename, locationfile):
    """
        Cone cone cone cone cone cone cone... 
        Cone cone. Cone cone cone cone cone.

        Spawn obstacles of type obstacletype with name obstaclename at locations indicated in location file Carla.Transform
    """
    try:
        obslocs = np.loadtxt(locationfile,delimiter=',')
    except:
        print("Error, File not Found")
        return
    
    cone_library = world.get_blueprint_library()
    cone = cone_library.find(obstacletype)
    cone.set_attribute('role_name', obstaclename)


    for i in range(obslocs.shape[0]):
        spawn = carla.Transform() #Test this on computer first
        spawn.location.x = obslocs[i,0]
        spawn.location.y = obslocs[i,1]
        spawn.location.z = obslocs[i,2]
        spawn.rotation.pitch = obslocs[i,3]
        spawn.rotation.yaw = obslocs[i,4]
        spawn.rotation.roll = obslocs[i,5]
        # print(spawn)
        this_cone = world.spawn_actor(cone,spawn)
        this_cone.set_enable_gravity(True) #Turn this back on
        this_cone.set_simulate_physics(True)
    return

def remove_CONE(world, obstaclename):
    """
        Remove all objects with obstaclename by turning off gravity and teleporting them 100 meters into the air.
    """

    actor_list = world.get_actors()
    for id in actor_list:
        try:
            if id.attributes["role_name"] == obstaclename:
                id.set_enable_gravity(False)
                cone_loc = id.get_location()
                cone_loc.z += 100
                id.set_location(cone_loc)
        except:
            continue

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
