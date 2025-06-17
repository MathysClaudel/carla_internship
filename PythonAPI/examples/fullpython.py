#!/usr/bin/env python3

import glob
import os
import sys
import random
import math
import struct

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


class PointCloudComplete:
    """
    Gère x,y,z,i,nx,ny,nz,cos,objectid,classid + timestamp
    """
    def __init__(self):
        self._xyz       = np.empty((0, 3), dtype=np.float32)
        self._intensity = np.empty((0,),     dtype=np.uint8)
        self._normals   = np.empty((0, 3),   dtype=np.float32)
        self._cosine    = np.empty((0,),     dtype=np.float32)
        self._id        = np.empty((0,),     dtype=np.uint32)
        self._tag       = np.empty((0,),     dtype=np.uint32)
        self._time      = np.empty((0,),     dtype=np.float32)

    def add_points(self, raw, timestamp):
        xyz       = raw[:, 0:3]
        i_uint8   = np.clip((raw[:, 3] * 255).round(), 0, 255).astype(np.uint8)
        normals   = raw[:, 4:7]
        cosine    = raw[:, 7]
        ids       = raw[:, 8].astype(np.uint32)
        tags      = raw[:, 9].astype(np.uint32)
        t_vec     = np.full((raw.shape[0],), timestamp, dtype=np.float32)

        self._xyz       = np.vstack((self._xyz,       xyz))
        self._intensity = np.concatenate((self._intensity, i_uint8))
        self._normals   = np.vstack((self._normals,   normals))
        self._cosine    = np.concatenate((self._cosine,     cosine))
        self._id        = np.concatenate((self._id,         ids))
        self._tag       = np.concatenate((self._tag,       tags))
        self._time      = np.concatenate((self._time,      t_vec))

    def save(self, path):
        n = self._xyz.shape[0]
        dtype = np.dtype([
            ('x',         'f4'), ('y', 'f4'), ('z', 'f4'),
            ('intensity', 'u1'),
            ('n_x',       'f4'), ('n_y', 'f4'), ('n_z', 'f4'),
            ('cosine',    'f4'),
            ('objectid',  'u4'), ('classid', 'u4'),
            ('time',      'f4'),
        ])
        vertices = np.empty(n, dtype=dtype)
        vertices['x']         = self._xyz[:, 0]
        vertices['y']         = self._xyz[:, 1]
        vertices['z']         = self._xyz[:, 2]
        vertices['intensity'] = self._intensity
        vertices['n_x']       = self._normals[:, 0]
        vertices['n_y']       = self._normals[:, 1]
        vertices['n_z']       = self._normals[:, 2]
        vertices['cosine']    = self._cosine
        vertices['objectid']  = self._id
        vertices['classid']   = self._tag
        vertices['time']      = self._time

        el = PlyElement.describe(vertices, 'vertex')
        PlyData([el], text=False).write(path)


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

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    # Synchrone 2 kHz
    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.0005
    world.apply_settings(settings)
    print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # Init Traffic Manager
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)

    blueprint_lib = world.get_blueprint_library()

    # Spawn véhicule principal (Tesla Model 3)
    main_bp = blueprint_lib.find('vehicle.tesla.model3')
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    main_spawn = spawn_points.pop()
    vehicle = world.spawn_actor(main_bp, main_spawn)
    print(f"Main vehicle (Tesla) at {main_spawn.location}")

    # Spawn trafic véhicules (40 aléatoires, excluant Lincoln/Tesla)
    all_vehicle_bps = blueprint_lib.filter('vehicle.*')
    all_vehicle_bps = [bp for bp in all_vehicle_bps if 'lincoln' not in bp.id]
    vehicle_bps = [bp for bp in all_vehicle_bps if bp.id != main_bp.id]
    traffic_vehicles = []
    for point in spawn_points[:40]:
        bp = random.choice(vehicle_bps)
        actor = world.try_spawn_actor(bp, point)
        if actor:
            actor.set_autopilot(True, tm.get_port())
            traffic_vehicles.append(actor)
    print(f"Spawned traffic vehicles: {len(traffic_vehicles)}")

    # Spawn piétons & vélos
    walker_bps = blueprint_lib.filter('walker.pedestrian.*')
    walker_controller_bp = blueprint_lib.find('controller.ai.walker')
    walker_controllers = []
    pedestrians = []
    for point in spawn_points[40:60]:
        bp = random.choice(walker_bps)
        walker = world.try_spawn_actor(bp, point)
        if walker:
            controller = world.spawn_actor(walker_controller_bp, carla.Transform(), attach_to=walker)
            controller.start()
            controller.go_to_location(world.get_random_location_from_navigation())
            controller.set_max_speed(1 + random.random())
            pedestrians.append(walker)
            walker_controllers.append(controller)
    print(f"Spawned pedestrians: {len(pedestrians)}, controllers: {len(walker_controllers)}")

    bike_bps = [bp for bp in all_vehicle_bps if 'motorcycle' in bp.id]
    bikes = []
    for point in spawn_points[60:70]:
        if not bike_bps:
            break
        bp = random.choice(bike_bps)
        bike = world.try_spawn_actor(bp, point)
        if bike:
            bike.set_autopilot(True, tm.get_port())
            tm.set_global_distance_to_leading_vehicle(bike, 2.0)
            tm.vehicle_percentage_speed_difference(bike, 30.0)
            bikes.append(bike)
    print(f"Spawned bikes: {len(bikes)}")

    # Warm-up
    for _ in range(int(1.0 / settings.fixed_delta_seconds)):
        world.tick()

    # Lancer autopilot du véhicule principal après warm-up
    vehicle.set_autopilot(True, tm.get_port())
    tm.ignore_lights_percentage(vehicle, 100.0)
    tm.ignore_signs_percentage(vehicle, 50.0)
    print("Autopilot principal activé")

    # IDs des acteurs à ignorer
    all_actors = traffic_vehicles + pedestrians + bikes
    ignore_all_ids  = [actor.id for actor in all_actors]         # ignore tous
    half = len(all_actors) // 2
    ignore_half_ids = [actor.id for actor in all_actors[:half]]  # ignore la moitié

    # --- Affichage dans la console de tous les acteurs et du mid traffic ---
    print("\n=== Liste de tous les acteurs (ID, Type) ===")
    for actor in all_actors:
        print(f"  • ID={actor.id}, Type={actor.type_id}")

    print("\n=== Liste des acteurs 'mid traffic' (ID, Type) ===")
    for actor in all_actors[:half]:
        print(f"  • ID={actor.id}, Type={actor.type_id}")
    print("=============================================\n")

    # Spawn trois LiDAR complets attachés au même véhicule
    lidar_bp = blueprint_lib.find('sensor.lidar.ray_cast_complete')
    params = {
        'range':              '100',
        'rotation_frequency': '20',
        'channels':           '32',
        'points_per_second':  '640000',
        'noise_stddev':       '0.02',
        'sensor_tick':        str(settings.fixed_delta_seconds),
        'upper_fov':          '15',
        'lower_fov':          '-15',
        'enable_ego_motion':  'true'
    }
    for k, v in params.items():
        lidar_bp.set_attribute(k, v)

    # Dossiers de sortie
    base_dir = os.path.expanduser('~/complete-data-traffic-modif/Town10/frames')
    dir_no   = os.path.join(base_dir, 'no_traffic')
    dir_mid  = os.path.join(base_dir, 'mid_traffic')
    dir_full = os.path.join(base_dir, 'full_traffic')
    os.makedirs(dir_no,  exist_ok=True)
    os.makedirs(dir_mid, exist_ok=True)
    os.makedirs(dir_full, exist_ok=True)

    # Nombre de mesures par révolution
    k = int(1 / (settings.fixed_delta_seconds * 20))  # 20 Hz de rotation

    # ---------------- LiDAR "no traffic" ----------------
    lidar_no = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    pc_no    = [PointCloudComplete()]  # liste pour être mutable dans callback
    count_no = [k - 1]
    rev_no   = [0]
    template_no = os.path.join(dir_no, 'scan_%08d.ply')

    def callback_no(meas):
        arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 10)
        ts  = meas.timestamp
        # Filtrer tous les points dont object_idx est dans ignore_all_ids
        raw_keep = [pt for pt in arr if int(pt[8]) not in ignore_all_ids]
        if raw_keep:
            raw_keep = np.stack(raw_keep, axis=0)
            pc_no[0].add_points(raw_keep, ts)
        count_no[0] += 1
        if count_no[0] == k:
            path = template_no % rev_no[0]
            pc_no[0].save(path)
            print(f"[no_traffic] Saved revolution #{rev_no[0]} ({pc_no[0]._xyz.shape[0]} pts) → {path}")
            rev_no[0]   += 1
            count_no[0] = 0
            pc_no[0]    = PointCloudComplete()

    lidar_no.listen(callback_no)

    # ---------------- LiDAR "mid traffic" ----------------
    lidar_mid = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    pc_mid    = [PointCloudComplete()]
    count_mid = [k - 1]
    rev_mid   = [0]
    template_mid = os.path.join(dir_mid, 'scan_%08d.ply')

    def callback_mid(meas):
        arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 10)
        ts  = meas.timestamp
        # Filtrer les points dont object_idx est dans ignore_half_ids
        raw_keep = [pt for pt in arr if int(pt[8]) not in ignore_half_ids]
        if raw_keep:
            raw_keep = np.stack(raw_keep, axis=0)
            pc_mid[0].add_points(raw_keep, ts)
        count_mid[0] += 1
        if count_mid[0] == k:
            path = template_mid % rev_mid[0]
            pc_mid[0].save(path)
            print(f"[mid_traffic] Saved revolution #{rev_mid[0]} ({pc_mid[0]._xyz.shape[0]} pts) → {path}")
            rev_mid[0]   += 1
            count_mid[0] = 0
            pc_mid[0]    = PointCloudComplete()

    lidar_mid.listen(callback_mid)

    # ---------------- LiDAR "full traffic" ----------------
    lidar_full = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    pc_full    = [PointCloudComplete()]
    count_full = [k - 1]
    rev_full   = [0]
    template_full = os.path.join(dir_full, 'scan_%08d.ply')

    def callback_full(meas):
        arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 10)
        ts  = meas.timestamp
        # Pas de filtrage
        pc_full[0].add_points(arr, ts)
        count_full[0] += 1
        if count_full[0] == k:
            path = template_full % rev_full[0]
            pc_full[0].save(path)
            print(f"[full_traffic] Saved revolution #{rev_full[0]} ({pc_full[0]._xyz.shape[0]} pts) → {path}")
            rev_full[0]   += 1
            count_full[0] = 0
            pc_full[0]    = PointCloudComplete()

    lidar_full.listen(callback_full)

    print("Trois LiDAR attachés et écoutent…")

    # Enregistrement pendant sim_time secondes
    sim_time  = 600.0
    num_ticks = int(sim_time / settings.fixed_delta_seconds)
    print(f"Recording for {sim_time}s → {num_ticks} ticks")

    trajectory = []

    for _ in range(num_ticks):
        world.tick()
        # Stockage de la trajectoire à chaque tick (pour LiDAR "full traffic")
        loc = vehicle.get_transform().location
        rot = vehicle.get_transform().rotation
        qx, qy, qz, qw = euler_to_quaternion(rot.roll, rot.pitch, rot.yaw)
        ts = world.get_snapshot().timestamp.elapsed_seconds
        vel = vehicle.get_velocity()
        acc = vehicle.get_acceleration()
        trajectory.append((loc.x, loc.y, loc.z, qx, qy, qz, qw,
                           ts, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z))

    # save trajectory
    out_dir_traj = os.path.expanduser('~/complete-data-traffic/Town10')
    os.makedirs(out_dir_traj, exist_ok=True)
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
    print("Cleaning up…")
    lidar_no.stop();   lidar_no.destroy()
    lidar_mid.stop();  lidar_mid.destroy()
    lidar_full.stop(); lidar_full.destroy()
    for actor in traffic_vehicles + pedestrians + bikes:
        actor.destroy()
    vehicle.destroy()
    for controller in walker_controllers:
        controller.stop()
        controller.destroy()
    for walker in pedestrians:
        walker.destroy()

    world.apply_settings(orig_settings)
    print("Done.")


if __name__ == '__main__':
    main()
