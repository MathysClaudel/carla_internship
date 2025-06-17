#!/usr/bin/env python3

import glob, os, sys, time
from queue import Queue
import numpy as np
import carla

# def write_ply_with_time(path, points, timestamp):
#     """
#     Écrit un tableau Nx4 (x,y,z,intensity) + timestamp en 5ᵉ colonne
#     points : numpy array N×4
#     timestamp : float seconds
#     """
#     N = points.shape[0]
#     header = [
#         "ply",
#         "format ascii 1.0",
#         f"element vertex {N}",
#         "property float x",
#         "property float y",
#         "property float z",
#         "property float intensity",
#         "property float time",
#         "end_header"
#     ]
#     with open(path, 'w') as f:
#         f.write("\n".join(header) + "\n")
#         for x, y, z, i in points:
#             f.write(f"{x:.6f} {y:.6f} {z:.6f} {i:.6f} {timestamp:.6f}\n")

def add_ply_with_time(path, points, timestamp):
    with open(path, 'a') as f:
        for x, y, z, i in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {i:.6f} {timestamp:.6f}\n")

def insert_ply(path):

    with open(path, 'r') as f:
        nb_points = sum(1 for _ in f)

    header = [
        "ply\n",
        "format ascii 1.0\n",
        f"element vertex {nb_points}\n",
        "property float x\n",
        "property float y\n",
        "property float z\n",
        "property float intensity\n",
        "property float time\n",
        "end_header\n"
    ]

    with open(path, 'r') as f:
        data_lines = f.readlines()

    with open(path, 'w') as f:
        f.writelines(header)
        f.writelines(data_lines)


def main():
    # 1) Connexion et world
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # # 2) Passer en synchrone 1 kHz
    # orig_settings = world.get_settings()
    # settings = world.get_settings()
    # settings.synchronous_mode    = True
    # settings.fixed_delta_seconds = 0.001
    # world.apply_settings(settings)
    # print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # 1) On démarre en mode ASYNCHRONE (c’est la valeur par défaut)
    settings = world.get_settings()
    settings.synchronous_mode = False
    world.apply_settings(settings)
    print("→ Mode asynchrone (collecte passive)…")

    # 3) Spawn du véhicule
    blueprint_library = world.get_blueprint_library()
    car_bp      = blueprint_library.find('vehicle.tesla.model3')
    spawn_point = carla.Transform(carla.Location(x=-93.1, y=27.9, z=2.0))
    vehicle     = world.spawn_actor(car_bp, spawn_point)

    # 1) Récupère l’objet Traffic Manager sur un port (ici 8000)
    tm = client.get_trafficmanager(8000)

    # 2) Lorsque tu actives l’autopilot, spécifie le même port TM :
    vehicle.set_autopilot(True, tm.get_port())

    # 3) Dis-lui d'IGNORER les feux à 100%
    #    (0 = obéir toujours, 100 = ignorer toujours)
    tm.ignore_lights_percentage(vehicle, 100)

    # (optionnel) Tu peux aussi régler d’autres comportements :
    tm.random_left_lanechange_percentage(vehicle, 0) 

    print(f"Spawned vehicle at {spawn_point.location}")

    time.sleep(5.0)


    # 6) Spawn du LiDAR “ring-par-tick”
    rotation_frequency = 10
    sensor_tick = 0.001 #DOIT ETRE EGAL AU DELTA_TIME_SIMULATE (LOGIQUE)


    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range',              '100')
    lidar_bp.set_attribute('rotation_frequency', str(rotation_frequency))     # 10 révolutions/s
    lidar_bp.set_attribute('channels',           '32')     # 32 anneaux
    lidar_bp.set_attribute('points_per_second',  '100000') # densité
    lidar_bp.set_attribute('noise_stddev',       '0')
    lidar_bp.set_attribute('sensor_tick',        str(sensor_tick))  # 1 ms simulé = 1 tick

    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))
    print("LiDAR attached and listening…")



    # 2) Puis on passe en mode SYNCHRONE
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.001   # ou la valeur que tu veux
    world.apply_settings(settings)
    print("→ Mode synchrone activé, fixed_delta_seconds =", settings.fixed_delta_seconds)

    # 7) Enregistrement pendant 8 s simulées
    sim_time  = 3.0
    num_ticks = int(sim_time / settings.fixed_delta_seconds)
    save_dir  = os.path.expanduser("~/Afull_trame_ts_em_autom_as/s_2")
    os.makedirs(save_dir, exist_ok=True)
    template  = os.path.join(save_dir, "ring_%06d.ply")
    print(f"Recording for {sim_time}s → {num_ticks} ticks")
    k=1/(sensor_tick * rotation_frequency)
    n=k-1

    for _ in range(num_ticks):
        world.tick()
        # on vide la file au besoin (normalement 1 frame LiDAR par tick)
        while not lidar_queue.empty():
            meas = lidar_queue.get()
            arr  = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1,4)
            ts   = meas.timestamp
            n=n+1
            if n%k == 0 :
                filename = template % meas.frame
                add_ply_with_time(filename,arr, ts)
            else :
                if n%k == k-1 :
                    add_ply_with_time(filename,arr, ts)
                    insert_ply(filename)
                else :
                    add_ply_with_time(filename,arr, ts)

    # 8) Cleanup
    print("Cleaning up…")
    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.apply_settings(orig_settings)
    print("Done.")

if __name__ == "__main__":
    main()
