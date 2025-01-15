# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time

try:
    this_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(glob.glob('%s/../carla/dist/carla-*%d.%d-%s.egg' % (
        this_dir,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
import numpy as np


def transform_to_homography(transform):
	