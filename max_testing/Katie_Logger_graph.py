#!/usr/bin/env python
import faulthandler; faulthandler.enable()

import glob
import os
import sys



import csv
import argparse
import random
import numpy as np
import time
import pygame
import math
import matplotlib.pyplot as plt

arr = np.loadtxt("KatieLogTest.csv",delimiter=",", dtype=str)
print(arr[0])

x = np.float64(arr[1:-9,2])
y = np.float64(arr[1:-9,3])
lap_count = (arr[1:-9,-2])
# x = np.float64(arr[-1000:,2])
# y = np.float64(arr[-1000:,3])
# collisions = arr[1:,-1]

x_hits = []
y_hits = []
object_collision = False
for i, col in enumerate(lap_count):
	if col != "None" and col != lap_count[35421] :
		print(col,i)
		x_hits.append(x[i])
		y_hits.append(y[i])
		object_collision = True

plt.figure()
plt.plot(x,y)

if object_collision:
	plt.scatter(x_hits,y_hits)
else:
	print("No Objects Hit")

plt.show()
# print(i)
plt.savefig("KatieLogTestGraph.png")
plt.close()