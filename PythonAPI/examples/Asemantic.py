#!/usr/bin/env python3

from plyfile import PlyData, PlyElement
import numpy as np
import carla
import os, glob, sys, time
from queue import Queue
import random
import math
import struct


class PointCloud:
    def __init__(self):
        self._xyz       = np.empty((0,3), dtype=np.float32)
        self._intensity = np.empty((0,),    dtype=np.uint16)
        self._time      = np.empty((0,),    dtype=np.float32)

    def add_points(self, xyz, intensity, timestamp):
        # Convertit intensity [0..1] -> uint16 [0..255]
        i_uint16 = np.clip((intensity * 255).round(), 0, 255).astype(np.uint16)
        t_vec = np.full((xyz.shape[0],), timestamp, dtype=np.float32)

        self._xyz       = np.vstack((self._xyz,       xyz.astype(np.float32)))
        self._intensity = np.concatenate((self._intensity, i_uint16))
        self._time      = np.concatenate((self._time,      t_vec))

    def save(self, path):
        # Prépare un array structuré pour PlyElement
        n = self._xyz.shape[0]
        vertex_dtype = np.dtype([
            ('x',        'f4'),
            ('y',        'f4'),
            ('z',        'f4'),
            ('intensity','u2'),
            ('time',     'f4'),
        ])
        vertices = np.empty(n, dtype=vertex_dtype)
        vertices['x']         = self._xyz[:,0]
        vertices['y']         = self._xyz[:,1]
        vertices['z']         = self._xyz[:,2]
        vertices['intensity'] = self._intensity
        vertices['time']      = self._time

        el = PlyElement.describe(vertices, 'vertex')
        PlyData([el], text=False).write(path) # mettre True pour ascii


trajectory = [] 

def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees → radians
    roll  = math.radians(roll)
    pitch = math.radians(pitch)
    yaw   = math.radians(yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def main():
    # 1) Connexion et world
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 2) Passer en synchrone 1 kHz
    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.0005
    world.apply_settings(settings)
    print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # 3) Spawn du véhicule
    blueprint_library = world.get_blueprint_library()
    car_bp      = blueprint_library.find('vehicle.tesla.model3')
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle     = world.spawn_actor(car_bp, spawn_point)

    # 4) Traffic Manager
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    vehicle.set_autopilot(True, tm.get_port())
    tm.ignore_lights_percentage(vehicle, 100)

    print(f"Spawned vehicle at {spawn_point.location}")

    # 5) Warm-up 0.5 s
    warmup_time  = 0.5
    warmup_ticks = int(warmup_time / settings.fixed_delta_seconds)
    for _ in range(warmup_ticks):
        world.tick()

    # Paramètres LiDAR
    rotation_frequency = 20
    sensor_tick = settings.fixed_delta_seconds

    # 6) Spawn du LiDAR sémantique
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
    lidar_bp.set_attribute('range',              '100')
    lidar_bp.set_attribute('rotation_frequency', str(rotation_frequency))
    lidar_bp.set_attribute('channels',           '32')
    lidar_bp.set_attribute('points_per_second',  '640000')
    lidar_bp.set_attribute('sensor_tick',        str(sensor_tick))
    lidar_bp.set_attribute('upper_fov',           '15')
    lidar_bp.set_attribute('lower_fov',           '-15')

    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_queue = Queue()
    lidar.listen(lambda meas: lidar_queue.put(meas))
    print("LiDAR sémantique attaché et en écoute…")

    # 7) Spawn du LiDAR classique
    lidar2_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar2_bp.set_attribute('range',              '100')
    lidar2_bp.set_attribute('rotation_frequency', str(rotation_frequency))
    lidar2_bp.set_attribute('channels',           '32')
    lidar2_bp.set_attribute('points_per_second',  '320000')
    lidar2_bp.set_attribute('noise_stddev',       '0.0')
    lidar2_bp.set_attribute('sensor_tick',        str(sensor_tick))
    lidar2_bp.set_attribute('upper_fov',           '15')
    lidar2_bp.set_attribute('lower_fov',           '-15')
    lidar2_bp.set_attribute('enable_ego_motion',   'false')

    lidar2 = world.spawn_actor(
        lidar2_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar2_queue = Queue()
    lidar2.listen(lambda data: lidar2_queue.put(data))
    print("LiDAR classique attaché et en écoute…")

    # 8) Enregistrement pendant sim_time
    sim_time  = 120.0
    num_ticks = int(sim_time / settings.fixed_delta_seconds)
    save_dir1  = os.path.expanduser("~/Data_Mathys_full/Data2/data1_semantic")
    save_dir2  = os.path.expanduser("~/Data_Mathys_full/Data2/data1")
    os.makedirs(save_dir1, exist_ok=True)
    os.makedirs(save_dir2, exist_ok=True)
    template1  = os.path.join(save_dir1, "trame_%08d.ply")
    template2  = os.path.join(save_dir2, "trame_%08d.ply")
    print(f"Recording for {sim_time}s → {num_ticks} ticks")

    k = int(1/(sensor_tick * rotation_frequency))
    n = k - 1
    rev_counter = 0

    pc  = PointCloud()
    pc2 = PointCloud()

    # Définition du dtype pour le Semantic LIDAR
    semantic_dtype = np.dtype([
        ('x',   'f4'), ('y', 'f4'), ('z', 'f4'),
        ('cos', 'f4'),
        ('inst','u4'), ('sem','u4'),
    ])

    for _ in range(num_ticks):
        world.tick()
        while not lidar_queue.empty() and not lidar2_queue.empty():
            meas  = lidar_queue.get()
            meas2 = lidar2_queue.get()
            ts    = meas.timestamp
            ts2   = meas2.timestamp

            # Parsing du Semantic LiDAR
            data = np.frombuffer(meas.raw_data, dtype=semantic_dtype)
            coords = np.stack([data['x'], data['y'], data['z']], axis=1)
            # cosines = data['cos']        # disponible si besoin
            # instance_ids = data['inst']  # disponible si besoin
            # semantic_tags = data['sem']  # disponible si besoin
            pc.add_points(coords, np.zeros(len(coords)), ts)

            # Parsing du LiDAR classique
            arr2 = np.frombuffer(meas2.raw_data, dtype=np.float32).reshape(-1,4)
            pc2.add_points(arr2[:,:3], arr2[:,3], ts2)

            n += 1
            if n % k == k-1:
                # Sauvegarde révolution
                filename1 = template1 % rev_counter
                pc.save(filename1)
                print(f"Saved semantic revolution #{rev_counter}")
                pc = PointCloud()

                filename2 = template2 % rev_counter
                pc2.save(filename2)
                print(f"Saved classic revolution #{rev_counter}")
                pc2 = PointCloud()

                rev_counter += 1

            # Enregistrement de la trajectoire
            transform = meas.transform
            loc       = transform.location
            rot       = transform.rotation
            qx, qy, qz, qw = euler_to_quaternion(rot.roll, rot.pitch, rot.yaw)
            trajectory.append((loc.x, loc.y, loc.z, qx, qy, qz, qw, ts))

    # Enregistrement final de la trajectoire en PLY
    with open(os.path.join(save_dir1, "trajectory3.ply"), "wb") as f:
        f.write(b"ply")
        f.write(b"format binary_little_endian 1.0")
        f.write(f"element vertex {len(trajectory)}".encode())
        f.write(b"property double x")
        f.write(b"property double y")
        f.write(b"property double z")
        f.write(b"property double q_x")
        f.write(b"property double q_y")
        f.write(b"property double q_z")
        f.write(b"property double q_w")
        f.write(b"property double timestamp")
        f.write(b"end_header")
        for pose in trajectory:
            f.write(struct.pack("<8d", *pose))

    # Cleanup
    print("Cleaning up…")
    lidar.stop()
    lidar.destroy()
    lidar2.stop()
    lidar2.destroy()
    vehicle.destroy()
    world.apply_settings(orig_settings)
    print("Done.")

if __name__ == "__main__":
    main()
