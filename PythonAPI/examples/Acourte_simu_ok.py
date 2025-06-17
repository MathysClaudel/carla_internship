#!/usr/bin/env python3

import glob, os, sys, time
from queue import Queue
import random
import numpy as np

# 1) Charger l’egg CARLA
try:
    egg = glob.glob(
        '../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'linux-x86_64'
        )
    )[0]
    sys.path.append(egg)
except IndexError:
    pass

import carla

def write_ply_with_time(path, points, timestamp):
    """
    Écrit un tableau Nx4 (x,y,z,intensity) + timestamp en 5ᵉ colonne
    points : numpy array N×4
    timestamp : float seconds
    """
    N = points.shape[0]
    header = [
        "ply",
        "format ascii 1.0",
        f"element vertex {N}",
        "property float x",
        "property float y",
        "property float z",
        "property float intensity",
        "property float time",
        "end_header"
    ]
    with open(path, 'w') as f:
        f.write("\n".join(header) + "\n")
        for x, y, z, i in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {i:.6f} {timestamp:.6f}\n")

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

    # 3) Mode synchrone à 1 kHz
    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.001
    world.apply_settings(settings)
    print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # 4) Spawn véhicule porteur
    blueprint_library = world.get_blueprint_library()
    car_bp   = blueprint_library.find('vehicle.tesla.model3')
    spawn_pt = random.choice(world.get_map().get_spawn_points())
    vehicle  = world.spawn_actor(car_bp, spawn_pt)
    vehicle.set_autopilot(True)
    print(f"Spawned vehicle at {spawn_pt.location}")

    # 5) Spawn LiDAR “anneau-par-tick”
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
    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))
    print("Spawned ring-by-tick LiDAR")

    # 6) Dossier de sauvegarde
    save_dir = os.path.expanduser("~/lidar_with_timestamps_world_courte")
    os.makedirs(save_dir, exist_ok=True)
    template = os.path.join(save_dir, "ring_%06d.ply")

    # 7) Boucle 5 s simulées
    sim_time  = 2
    dt        = settings.fixed_delta_seconds
    num_ticks = int(sim_time / dt)
    print(f"Running {sim_time}s sim → {num_ticks} ticks")

    for _ in range(num_ticks):
        world.tick()
        # tant que des mesures arrivent…
        while not lidar_queue.empty():
            meas = lidar_queue.get()
            arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1,4)
            # === on récupère directement le float ===
            ts = meas.timestamp  
            filename = template % meas.frame
            write_ply_with_time(filename, arr, ts)
            print(f"Saved {filename}: {arr.shape[0]} pts @ t={ts:.3f}s")

    # 8) Cleanup
    print("Cleaning up…")
    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.apply_settings(orig_settings)
    print("Done.")

if __name__ == "__main__":
    main()
