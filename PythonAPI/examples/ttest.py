import glob
import os
import sys
import random
import math
import struct
from queue import Queue

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
from plyfile import PlyData, PlyElement
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Cherchez le blueprint « ray_cast_complete »
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_complete')
print('Found blueprint:', lidar_bp.id)

# Spawn un LiDAR simple pour tester
ego_vehicle = world.spawn_actor(world.get_blueprint_library().filter('vehicle.*')[0],
                                random.choice(world.get_map().get_spawn_points()))
lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=2.5)), attach_to=ego_vehicle)

# Maintenant :
print(dir(lidar))  
# Voyez si 'set_ignored_actor_ids' figure dans la liste.
