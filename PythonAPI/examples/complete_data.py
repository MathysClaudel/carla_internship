#!/usr/bin/env python3

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
from plyfile import PlyData, PlyElement
import numpy as np
import math

import random
import time


def main():
    # 1) Connexion
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 2) Mode synchrone à 2 kHz
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.0005
    world.apply_settings(settings)
    print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # 3) Spawn véhicule
    blueprint_lib = world.get_blueprint_library()
    car_bp = blueprint_lib.find('vehicle.tesla.model3')
    # spawn_point = carla.Transform(carla.Location(x=55.5, y=-57.3, z=0.2))
    # vehicle = world.spawn_actor(car_bp, spawn_point)
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle  = world.spawn_actor(car_bp, spawn_point)
    vehicle.set_autopilot(True)
    print("Vehicle spawned and autopilot on")

    # 4) Spawn LiDAR complet
    lidar_bp = blueprint_lib.find('sensor.lidar.ray_cast_complete')
    lidar_bp.set_attribute('range',              '100')
    lidar_bp.set_attribute('rotation_frequency', '20')
    lidar_bp.set_attribute('channels',           '32')
    lidar_bp.set_attribute('points_per_second',  '640000')
    lidar_bp.set_attribute('noise_stddev',       '0.02')
    lidar_bp.set_attribute('sensor_tick',        str(settings.fixed_delta_seconds))
    lidar_bp.set_attribute('upper_fov',          '15')
    lidar_bp.set_attribute('lower_fov',          '-15')
    lidar_bp.set_attribute('enable_ego_motion',  'false')

    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )

    from queue import Queue

    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))
    print("Complete LiDAR attached and listening…")

    # 5) Warm-up
    for _ in range(10):
        world.tick()

    # 6) Enregistrement simple de 5 trames
    os.makedirs('lidar_out', exist_ok=True)
    for i in range(5):
        world.tick()
        meas = lidar_queue.get(timeout=1.0)
        points = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1,10)
        # x,y,z,i,nx,ny,nz,cos,id,tag
        np.savetxt(f'lidar_out/scan_{i:03d}.txt', points, fmt='%.6f')
        print(f"Saved scan {i}")
        print(f"Scan {i:03d}: {points.shape[0]} points capturés")
        print("  Premier point :", points[0])
        print("  Dernier point :", points[-1])

    # 7) Cleanup
    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.apply_settings(world.get_settings())  # remet ancien réglage
    print("Done.")

if __name__ == '__main__':
    main()
