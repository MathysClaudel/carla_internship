
import glob, os, sys, random, math, struct
from queue import Queue
import time 

# Insert CARLA egg
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np
import array
from plyfile import PlyData, PlyElement

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)

maps = client.get_available_maps()
for m in maps:
    print(m)