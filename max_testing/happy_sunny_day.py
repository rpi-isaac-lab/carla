#!/usr/bin/env python

# Blatantly stolen from Carla

"""
CARLA Happy Sunny Day:

Connect to a Carla Simulator Instance to set sun at high elevation with no clouds/rain/etc.
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
import math


def make_nice_weather(world):
  weather = world.get_weather()
  weather.sun_azimuth_angle = 0
  weather.sun_altitude_angle = 60
  weather.cloudiness = 0
  weather.precipitation = 0
  weather.precipitation_deposits = 0
  weather.wind_intensity =0
  weather.fog_density = 0
  weather.wetness = 0
  world.set_weather(weather)


def main():
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

    make_nice_weather(world)


if __name__ == '__main__':

    main()
