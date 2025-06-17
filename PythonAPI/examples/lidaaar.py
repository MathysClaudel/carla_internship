#!/usr/bin/env python

import glob
import os
import sys
import time
from queue import Queue

try:
    sys.path.append(glob.glob(
        '../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'
        )
    )[0])
except IndexError:
    pass

import carla
import random

def main():
    client = None
    world = None
    original_settings = None
    vehicle = None
    lidar = None

    try:
        # 1) Connexion et attente du serveur CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        while True:
            try:
                world = client.get_world()
                break
            except RuntimeError:
                print("Waiting for CARLA…")
                time.sleep(1.0)

        # 2) Passer en mode synchrone à 1 kHz
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.001  # 1 ms
        world.apply_settings(settings)

        # 3) Spawn d’un véhicule pour porter le LiDAR
        blueprint_library = world.get_blueprint_library()
        car_bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(car_bp, spawn_point)
        vehicle.set_autopilot(True)
        print(f"Spawned vehicle at {spawn_point.location}")

        # 4) Création du LiDAR « beam-by-beam »
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range',             '100')
        lidar_bp.set_attribute('rotation_frequency','20')
        lidar_bp.set_attribute('channels',          '32')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('noise_stddev',      '0')
        lidar_bp.set_attribute('sensor_tick',       '0.001')  # publication tous les 1 ms

        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.0))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        print("Spawned ray-cast LiDAR")

        # 5) Queue et dossier de sauvegarde
        lidar_queue = Queue()
        lidar.listen(lambda data: lidar_queue.put(data))

        home = os.path.expanduser("~")
        save_dir = os.path.join(home, "lidar_ray_cast4")
        os.makedirs(save_dir, exist_ok=True)
        path_template = os.path.join(save_dir, "scan_%06d.ply")

        # 6) Boucle de simulation : 5 s simulées → 5000 ticks
        sim_time = 5.0
        tick_dt  = settings.fixed_delta_seconds
        num_ticks = int(sim_time / tick_dt)
        print(f"Running {sim_time}s sim → {num_ticks} ticks at {1/tick_dt:.0f} Hz")

        for _ in range(num_ticks):
            world.tick()
            if not lidar_queue.empty():
                measurement = lidar_queue.get()
                filename = path_template % measurement.frame
                measurement.save_to_disk(filename)

    finally:
        # 7) Cleanup
        print("Stopping and destroying actors…")
        if lidar is not None:
            lidar.stop()
        if vehicle is not None:
            vehicle.destroy()
        if world and original_settings:
            world.apply_settings(original_settings)
        print("Done.")

if __name__ == "__main__":
    main()
