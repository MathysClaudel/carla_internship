#!/usr/bin/env python3

import glob, os, sys, time
from queue import Queue
import random

# 1) Charger l’egg CARLA
try:
    sys.path.append(glob.glob(
        '../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'linux-x86_64'
        )
    )[0])
except IndexError:
    pass

import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # 2) Attendre que CARLA réponde
    while True:
        try:
            world = client.get_world()
            break
        except RuntimeError:
            print("Waiting for CARLA…")
            time.sleep(1.0)

    # 3) Passer en mode synchrone à 1 kHz
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.001   # 1 ms/tick
    world.apply_settings(settings)
    print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # 4) Spawn d’un véhicule porteur
    blueprint_library = world.get_blueprint_library()
    car_bp   = blueprint_library.find('vehicle.tesla.model3')
    spawn_pt = random.choice(world.get_map().get_spawn_points())
    vehicle  = world.spawn_actor(car_bp, spawn_pt)
    vehicle.set_autopilot(True)
    print(f"Spawned vehicle at {spawn_pt.location}")

    # 5) Spawn du LiDAR “anneau-par-tick”
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range',              '100')
    lidar_bp.set_attribute('rotation_frequency', '10')
    lidar_bp.set_attribute('channels',           '32')
    lidar_bp.set_attribute('points_per_second',  '100000')
    lidar_bp.set_attribute('noise_stddev',       '0')
    lidar_bp.set_attribute('sensor_tick',        '0.001')

    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle)
    print("Spawned ring-by-tick LiDAR")

    # 6) Préparer la queue et le dossier où sauver
    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))

    save_dir = os.path.expanduser("~/lidar_ring_by_tick2")
    os.makedirs(save_dir, exist_ok=True)
    path_template = os.path.join(save_dir, "ring_%06d.ply")

    # 7) Boucle 5 s simulées
    sim_time   = 5.0
    dt         = settings.fixed_delta_seconds
    num_ticks  = int(sim_time / dt)
    print(f"Running {sim_time}s sim → {num_ticks} ticks")

    for _ in range(num_ticks):
        world.tick()
        # Sauver chaque anneau reçu (contient `channels` points)
        while not lidar_queue.empty():
            meas = lidar_queue.get()
            filename = path_template % meas.frame
            meas.save_to_disk(filename)
            print(f"Saved ring frame={meas.frame} points={len(meas)}")

    # 8) Cleanup
    print("Cleaning up…")
    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.apply_settings(original_settings)
    print("Done.")

if __name__ == "__main__":
    main()
