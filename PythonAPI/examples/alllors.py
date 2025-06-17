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


def write_ply_with_time(path, cloud):
    """
    Écrit un tableau Nx5 (x,y,z,intensity,time) dans un PLY ascii.
    cloud : numpy array shape (N,5)
    """
    N = cloud.shape[0]
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
        for x, y, z, i, t in cloud:
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {i:.6f} {t:.6f}\n")

def main():
    # 1) Connexion et settings CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    orig = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.001
    world.apply_settings(settings)
    dt = settings.fixed_delta_seconds
    print(f"Synchronous @ {1/dt:.0f} Hz")

    # 2) Spawn véhicule et vitesse
    blueprint_library = world.get_blueprint_library()
    car_bp      = blueprint_library.find('vehicle.tesla.model3')
    spawn_point = carla.Transform(carla.Location(x=-93.1, y=27.9, z=2.0))
    vehicle     = world.spawn_actor(car_bp, spawn_point)
    speed_m_s   = 100.0 / 3.6
    fv = vehicle.get_transform().get_forward_vector()
    vehicle.set_target_velocity(carla.Vector3D(fv.x * speed_m_s,
                                               fv.y * speed_m_s,
                                               fv.z * speed_m_s))
    print(f"Spawned vehicle at {spawn_point.location}, speed={speed_m_s:.2f} m/s")

    # 3) Warm-up 1 s
    for _ in range(int(1.0 / dt)):
        world.tick()

    # 4) Spawn LiDAR
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range',              '100')
    lidar_bp.set_attribute('rotation_frequency', '10')      # 10 Hz → 0.1 s/rev
    lidar_bp.set_attribute('channels',           '32')
    lidar_bp.set_attribute('points_per_second',  '100000')
    lidar_bp.set_attribute('noise_stddev',       '0')
    lidar_bp.set_attribute('sensor_tick',        '0.001')   # publication every tick
    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle)
    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))
    print("LiDAR attached and listening…")

    # 5) Préparation du buffer et du dossier
    sim_time      = 8.0
    num_ticks     = int(sim_time / dt)
    save_dir      = os.path.expanduser("~/100")
    os.makedirs(save_dir, exist_ok=True)
    tpl_full      = os.path.join(save_dir, "fullrev_%04d.ply")
    rings_per_rev = 100  # 1/(10Hz*0.001s) = 100 rings per revolution
    ring_buffer   = []   # contiendra des tuples (pts_array, timestamp)
    rev_counter   = 0

    print(f"Recording {sim_time}s → {num_ticks} ticks, "
          f"{rings_per_rev} rings/revolution")

    # 6) Boucle principale
    for _ in range(num_ticks):
        world.tick()
        while not lidar_queue.empty():
            meas = lidar_queue.get()
            pts  = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1,4)
            ts   = meas.timestamp

            # Accumule chaque anneau avec son timestamp
            ring_buffer.append((pts, ts))

            # Dès qu'on a atteint une révolution complète :
            if len(ring_buffer) >= rings_per_rev:
                # Concatène les anneaux en un grand nuage Nx4
                clouds  = [rb[0] for rb in ring_buffer[:rings_per_rev]]
                full_pts = np.vstack(clouds)  # shape (N,4)

                # Concatène les timestamps en un vecteur Nx1
                times = np.vstack([
                    np.full((c.shape[0],1), t)
                    for c, t in ring_buffer[:rings_per_rev]
                ])  # shape (N,1)

                # Construit le nuage Nx5
                full_cloud = np.hstack([full_pts, times])  # shape (N,5)

                # Écrit le PLY avec timestamp par point
                filename = tpl_full % rev_counter
                write_ply_with_time(filename, full_cloud)
                print(f"Saved full revolution #{rev_counter}: {full_cloud.shape[0]} pts")

                rev_counter += 1
                # Retire les anneaux déjà consommés
                ring_buffer = ring_buffer[rings_per_rev:]

    # 7) Cleanup
    print("Cleaning up…")
    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.apply_settings(orig)
    print("Done.")

if __name__ == "__main__":
    main()
