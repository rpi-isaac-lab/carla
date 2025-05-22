#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

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

import random
import time


def main():
	actor_list = []

	# In this script, I will spawn an automated vehicle and track its state data

	try:
		# First of all, we need to create the client that will send the requests
		# to the simulator. Here we'll assume the simulator is accepting
		# requests in the localhost at port 2000.
		client = carla.Client('localhost', 2000)
		client.set_timeout(2.0)

		# Once we have a client we can retrieve the world that is currently
		# running.
		world = client.get_world()

		# The world contains the list blueprints that we can use for adding new
		# actors into the simulation.
		blueprint_library = world.get_blueprint_library()

		# Time to grab a car!
		## Let's list the vehicles
		for vehicle in blueprint_library.filter('vehicle'):
			print(vehicle.id)

		return
		## Let's grab the prius
		prius = blueprint_library.find("vehicle.toyota.prius")
		### Let's look at the recommended paint colors
		for color in prius.get_attribute("color").recommended_values:
			print(color)
		### Let's paint it red
		red = "255,0,0"
		prius.set_attribute('color',red)

		# Let's find a nice place to drop it
		## Take a look at all of our options
		for i,spawn in enumerate(world.get_map().get_spawn_points()):
			print("Spawn {}: {}".format(i,spawn))
		## Just take the zeroth one
		spawn = world.get_map().get_spawn_points()[0]

		# Actually spawn the car and set autopilot to true
		vehicle = world.spawn_actor(prius, spawn)
		actor_list.append(vehicle)
		print('created %s' % vehicle.type_id)
		vehicle.set_autopilot(True)

		# Let it drive around a bit and record the data
		t_max = 50 # Seconds
		t_start = time.time()

		# Set file for reading


		while time.time()-t_start < t_max:
			print("Prius State:")
			state = vehicle.get_transform()
			print("\tPosition: {}".format(state.location))
			print("\tRotation: {}".format(state.rotation))
			print("-"*40)
			time.sleep(1)

	finally:

		print('destroying actors')
		client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
		print('done.')


if __name__ == '__main__':
	main()
