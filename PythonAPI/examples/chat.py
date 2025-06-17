#!/usr/bin/env python3

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

# -------------------------------------------------------------------
#                AJOUT CAMERA & COLOR FUSION
# -------------------------------------------------------------------

# File pour stocker les images RGB
latest_image = None

def camera_callback(image):
    """Stocke en numpy array la dernière frame RGB (Height×Width×3)."""
    global latest_image
    # Données BGRA 8 bits
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    # On prend R,G,B et on inverse de BGR->RGB
    latest_image = array[:, :, :3][:, :, ::-1]

# -------------------------------------------------------------------

class PointCloudComplete:
    """
    Gère x,y,z,i,nx,ny,nz,cos,objectid,classid + timestamp + RGB
    """
    def __init__(self):
        self._xyz       = np.empty((0,3), dtype=np.float32)
        self._intensity = np.empty((0,),    dtype=np.uint8)
        self._normals   = np.empty((0,3),   dtype=np.float32)
        self._cosine    = np.empty((0,),    dtype=np.float32)
        self._id        = np.empty((0,),    dtype=np.uint32)
        self._tag       = np.empty((0,),    dtype=np.uint32)
        self._time      = np.empty((0,),    dtype=np.float32)
        # Nouveaux tableaux pour la couleur
        self._r         = np.empty((0,),    dtype=np.uint8)
        self._g         = np.empty((0,),    dtype=np.uint8)
        self._b         = np.empty((0,),    dtype=np.uint8)

    def add_points(self, raw, timestamp, cam):
        """
        raw: ndarray Nx10 = [X,Y,Z,I,nx,ny,nz,cos,objectid,classid]
        """
        # Extractions existantes :
        xyz       = raw[:, 0:3]
        i_uint8   = np.clip((raw[:, 3] * 255).round(), 0, 255).astype(np.uint8)
        normals   = raw[:, 4:7]
        cosine    = raw[:, 7]
        ids       = raw[:, 8].astype(np.uint32)
        tags      = raw[:, 9].astype(np.uint32)
        t_vec     = np.full((raw.shape[0],), timestamp, dtype=np.float32)
        r      = raw[:, 10].astype(np.uint8)
        g      = raw[:, 11].astype(np.uint8)
        b      = raw[:, 12].astype(np.uint8)

        # Stockage standard
        self._xyz       = np.vstack((self._xyz,       xyz))
        self._intensity = np.concatenate((self._intensity, i_uint8))
        self._normals   = np.vstack((self._normals,   normals))
        self._cosine    = np.concatenate((self._cosine,    cosine))
        self._id        = np.concatenate((self._id,        ids))
        self._tag       = np.concatenate((self._tag,       tags))
        self._time      = np.concatenate((self._time,      t_vec))
        self._r = np.concatenate((self._r, r))
        self._g = np.concatenate((self._g, g))
        self._b = np.concatenate((self._b, b))
        # --- fin fusion couleur ---


    def save(self, path):
        n = self._xyz.shape[0]
        dtype = np.dtype([
            ('x',         'f4'), ('y',         'f4'), ('z',         'f4'),
            ('intensity', 'u1'),
            ('n_x',       'f4'), ('n_y',       'f4'), ('n_z',       'f4'),
            ('cosine',    'f4'),
            ('objectid',  'u4'), ('classid',   'u4'),
            ('red',       'u1'), ('green',     'u1'), ('blue', 'u1'),
            ('time',      'f4'),
        ])
        vertices = np.empty(n, dtype=dtype)
        vertices['x']         = self._xyz[:,0]
        vertices['y']         = self._xyz[:,1]
        vertices['z']         = self._xyz[:,2]
        vertices['intensity'] = self._intensity
        vertices['n_x']       = self._normals[:,0]
        vertices['n_y']       = self._normals[:,1]
        vertices['n_z']       = self._normals[:,2]
        vertices['cosine']    = self._cosine
        vertices['objectid']  = self._id
        vertices['classid']   = self._tag
        vertices['red']       = self._r
        vertices['green']     = self._g
        vertices['blue']      = self._b
        vertices['time']      = self._time

        el = PlyElement.describe(vertices, 'vertex')
        PlyData([el], text=False).write(path)

# -------------------------------------------------------------------

def euler_to_quaternion(roll, pitch, yaw):
    roll  = math.radians(roll)
    pitch = math.radians(pitch)
    yaw   = math.radians(yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw

def main():
    # 1) Connexion et settings CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    world = client.load_world('Town03')
    orig = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.0005
    world.apply_settings(settings)
    dt = settings.fixed_delta_seconds
    print(f"Synchronous @ {1/dt:.0f} Hz")

    # 2) Initialisation du Traffic Manager
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)  # on reste en mode synchrone côté TM
    # (optionnel) on peut faire en sorte que le TM ignore feux et panneaux si besoin :
    # tm.ignore_lights_percentage(100.0)
    # tm.ignore_signs_percentage(100.0)

    # 3) Spawn véhicule
    blueprint_library = world.get_blueprint_library()
    car_bp      = blueprint_library.find('vehicle.tesla.model3')
    spawn_point = carla.Transform(
        carla.Location(x=-93.1, y=27.9, z=2.0),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    )
    vehicle = world.spawn_actor(car_bp, spawn_point)
    print(f"Spawned vehicle at {spawn_point.location}")

    # 4) Activation de l’autopilot sur ce véhicule via le Traffic Manager
    vehicle.set_autopilot(True, tm.get_port())

    # 5) Forcer la vitesse désirée à 100 km/h
    tm.set_desired_speed(vehicle, 100.0)

    print("Autopilot activé → vitesse forcée à 100 km/h")

    # --- Spawn et écoute de la caméra RGB ---
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '800')
    cam_bp.set_attribute('image_size_y', '600')
    cam_bp.set_attribute('fov', '90')
    cam = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(x=0.0, z=2.5)),
        attach_to=vehicle)
    cam.listen(camera_callback)

    # Spawn LiDAR complet et écoute
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_complete')
    params = {
        'range':             '100',
        'rotation_frequency':'20',
        'channels':          '32',
        'points_per_second': '640000',
        'noise_stddev':      '0.02',
        'sensor_tick':       str(settings.fixed_delta_seconds),
        'upper_fov':         '15',
        'lower_fov':         '-15',
        'enable_ego_motion': 'true'
    }
    for k, v in params.items(): lidar_bp.set_attribute(k, v)
    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))
    print("LiDAR complet attaché…")

    # Enregistrement
    sim_time  = 600.0
    num_ticks = int(sim_time / settings.fixed_delta_seconds)
    out_dir   = os.path.expanduser('~/complete-data-traffic/25_06_02_lundi/frames5')
    os.makedirs(out_dir, exist_ok=True)
    template  = os.path.join(out_dir, 'scan_%08d.ply')
    print(f"Recording for {sim_time}s → {num_ticks} ticks")

    k = int(1/(settings.fixed_delta_seconds * 20))
    n = k - 1
    rev = 0
    pc = PointCloudComplete()
    trajectory = []

    # Boucle de simulation
    for _ in range(num_ticks):
        world.tick()
        while not lidar_queue.empty():
            meas = lidar_queue.get()
            arr  = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1,13)
            ts   = meas.timestamp

            # On passe la camera en argument pour la fusion
            pc.add_points(arr, ts, cam)

            vel  = lidar.get_velocity()
            acc  = lidar.get_acceleration()
            loc   = meas.transform.location
            rot   = meas.transform.rotation
            qx,qy,qz,qw = euler_to_quaternion(rot.roll, rot.pitch, rot.yaw)
            trajectory.append((loc.x, loc.y, loc.z, qx, qy, qz, qw,
                               ts, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z))

            n += 1
            if n == k:
                path = template % rev
                pc.save(path)
                print(f"Saved revolution #{rev} → {pc._xyz.shape[0]} pts → {path}")
                rev += 1
                n = 0
                pc = PointCloudComplete()

    # save trajectory
    out_dir_traj   = os.path.expanduser('~/100')
    traj_file = os.path.join(out_dir_traj, 'trajectory.ply')
    with open(traj_file, 'wb') as f:
        f.write(b"ply\n")
        f.write(b"format binary_little_endian 1.0\n")
        f.write(f"element vertex {len(trajectory)}\n".encode())
        for prop in ['x','y','z','q_x','q_y','q_z','q_w',
                     'timestamp','vel_x','vel_y','vel_z','acc_x','acc_y','acc_z']:
            f.write(f"property double {prop}\n".encode())
        f.write(b"end_header\n")
        for p in trajectory:
            f.write(struct.pack("<14d", *p))
    print(f"Trajectory saved → {traj_file}")

    # Cleanup
    lidar.stop(); lidar.destroy()
    cam.stop();   cam.destroy()
    vehicle.destroy()
    world.apply_settings(orig)
    print("Done.")

if __name__ == '__main__':
    main()
