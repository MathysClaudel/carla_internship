#!/usr/bin/env python3

import glob, os, sys, random, math, struct
from queue import Queue
import time 

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
import array
from plyfile import PlyData, PlyElement


class PointCloudComplete:
    """
    Gère x,y,z,i,nx,ny,nz,cos,objectid,classid + timestamp
    """
    def __init__(self):
        self._xyz       = np.empty((0,3), dtype=np.float32)
        self._intensity = np.empty((0,),    dtype=np.uint8)
        self._normals   = np.empty((0,3),   dtype=np.float32)
        self._cosine    = np.empty((0,),    dtype=np.float32)
        self._id        = np.empty((0,),    dtype=np.uint32)
        self._tag       = np.empty((0,),    dtype=np.uint32)
        self._time      = np.empty((0,),    dtype=np.float32)

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
        self._cosine    = np.concatenate((self._cosine,    cosine))
        self._id        = np.concatenate((self._id,        ids))
        self._tag       = np.concatenate((self._tag,       tags))
        self._time      = np.concatenate((self._time,      t_vec))

    def save(self, path):
        n = self._xyz.shape[0]
        dtype = np.dtype([
            ('x',         'f4'), ('y',         'f4'), ('z',         'f4'),
            ('intensity', 'u1'),
            ('n_x',       'f4'), ('n_y',       'f4'), ('n_z',       'f4'),
            ('cosine',    'f4'),
            ('objectid',  'u4'), ('classid',   'u4'),
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

    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 1) Passer la scène en mode synchrone à 2000 Hz (0.0005 s)
    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = 0.0005
    world.apply_settings(settings)
    print(f"Synchronous @ {1/settings.fixed_delta_seconds:.0f} Hz")

    # 2) Initialiser le Traffic Manager
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)

    blueprint_lib = world.get_blueprint_library()

    # 3) Spawn du véhicule principal (Tesla Model 3)
    main_bp      = blueprint_lib.find('vehicle.tesla.model3')
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    main_spawn = spawn_points.pop()
    vehicle    = world.spawn_actor(main_bp, main_spawn)
    print(f"Main vehicle (Tesla) at {main_spawn.location}")

    # 4) Spawn du trafic : 40 véhicules, 20 piétons, 10 motos
    all_vehicle_bps = blueprint_lib.filter('vehicle.*')
    all_vehicle_bps = [bp for bp in all_vehicle_bps if 'lincoln' not in bp.id]
    vehicle_bps     = [bp for bp in all_vehicle_bps if bp.id != main_bp.id]

    traffic_vehicles = []
    for point in spawn_points[:80]:
        bp = random.choice(vehicle_bps)
        actor = world.try_spawn_actor(bp, point)
        if actor:
            actor.set_autopilot(True, tm.get_port())
            traffic_vehicles.append(actor)
    print(f"Spawned traffic vehicles: {len(traffic_vehicles)}")

    # 5) Activer l’autopilot de l’ego (et config TM)
    vehicle.set_autopilot(True, tm.get_port())
    tm.ignore_lights_percentage(vehicle, 100.0)
    tm.ignore_signs_percentage(vehicle, 50.0)
    print("Autopilot principal activé")

    # 6) Préparer les listes d’IDs à ignorer
    all_ids         = [a.id for a in traffic_vehicles]
    all_ids = [int(x) for x in all_ids]
    # print("=== Contenu de ignore_all_ids et leurs types ===")
    # for idx, x in enumerate(all_ids):
    #     print(f"[{idx}] = {x!r} (type: {type(x)})")
    # print("===============================================")

    half            = len(all_ids) // 2
    ignore_all_ids  = all_ids 
    ignore_half_ids = all_ids[:half] 
    ignore_none_ids = []

    # 7) Créer le blueprint LiDAR “ray_cast_complete”
    lidar_bp = blueprint_lib.find('sensor.lidar.ray_cast_complete')
    params   = {
        'range':              '90',
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

    # 8) Spawn des trois LiDAR (no/mid/full) et configuration set_ignored_actors
    # -----------------------------------------------------------------------------
    # a) LiDAR “no_traffic” (ignore TOUS les actors)
    lidar_no = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_no.set_ignored_actors(ignore_all_ids)
    print(f"LiDAR no_traffic ignore {len(ignore_all_ids)} actors:")
    for actor_id in ignore_all_ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            print(f"  - ID {actor_id} → {actor.type_id}")
        else:
            print(f"  - ID {actor_id} → <acteur non trouvé>")

    # b) LiDAR “mid_traffic” (ignore la moitié seulement)
    lidar_mid = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_mid.set_ignored_actors(ignore_half_ids)
    print(f"LiDAR mid_traffic ignore {len(ignore_half_ids)} actors:")
    for actor_id in ignore_half_ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            print(f"  - ID {actor_id} → {actor.type_id}")
        else:
            print(f"  - ID {actor_id} → <acteur non trouvé>")

    # c) LiDAR “full_traffic” (ignore personne)
    lidar_full = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_full.set_ignored_actors(ignore_none_ids)
    print("LiDAR full_traffic ignore 0 actors")


    timestr = time.strftime("%Y%m%d-%H%M%S")
    # 9) Créer les dossiers de sortie pour chaque LiDAR
    base_dir = os.path.expanduser(f'~/dataset/Town10_{timestr}')
    os.makedirs(base_dir, exist_ok=True)
    frame_dir = os.path.join(base_dir, 'frames')
    dir_no   = os.path.join(frame_dir, 'no_traffic')
    dir_mid  = os.path.join(frame_dir, 'mid_traffic')
    dir_full = os.path.join(frame_dir, 'full_traffic')
    os.makedirs(frame_dir, exist_ok=True)
    os.makedirs(dir_no,  exist_ok=True)
    os.makedirs(dir_mid, exist_ok=True)
    os.makedirs(dir_full, exist_ok=True)

    # 10) Déterminer le nombre de mesures par révolution (20 Hz → k ticks)
    k = int(1 / (settings.fixed_delta_seconds * 20))  # ex: 0.0005 * 20 = 0.01 → 1/0.01 = 100

    # 11) Préparer les structures pour accumuler les points et compter les révolutions
    # -----------------------------------------------------------------------------
    # -- LiDAR no_traffic --
    pc_no    = [PointCloudComplete()]
    count_no = [k - 1]
    rev_no   = [0]
    template_no = os.path.join(dir_no, 'scan_%08d.ply')

    def callback_no(meas):
        arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 10)
        ts  = meas.timestamp
        # Filtrer tout point dont object_idx est dans ignore_all_ids
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


    # -- LiDAR mid_traffic --
    pc_mid    = [PointCloudComplete()]
    count_mid = [k - 1]
    rev_mid   = [0]
    template_mid = os.path.join(dir_mid, 'scan_%08d.ply')

    def callback_mid(meas):
        arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 10)
        ts  = meas.timestamp
        # Filtrer tout point dont object_idx est dans ignore_half_ids
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


    # -- LiDAR full_traffic --
    pc_full    = [PointCloudComplete()]
    count_full = [k - 1]
    rev_full   = [0]
    template_full = os.path.join(dir_full, 'scan_%08d.ply')

    def callback_full(meas):
        arr = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 10)
        ts  = meas.timestamp
        # Pas de filtrage : on prend tous les points
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


    # 11.bis) Create the Ground Truth trajectory ply file beforehand
    traj_file_ply = os.path.join(base_dir, 'trajectory_gt.ply')
    traj_file_tum = os.path.join(base_dir, 'trajectory_gt.tum')

    with open(traj_file_ply, 'wb') as f:
        f.write(b"ply\n")
        f.write(b"format binary_little_endian 1.0\n")
        f.write(f"element vertex {0:>8}\n".encode())
        for prop in ['x','y','z','q_x','q_y','q_z','q_w','timestamp','vel_x','vel_y','vel_z','acc_x','acc_y','acc_z']:
            f.write(f"property double {prop}\n".encode())
        f.write(b"end_header\n")

    # Function to update in-place the number of vertices of the ply files
    def update_ply_elements_number(file_path, elements_number):
        with open(file_path, 'r+b') as f:
            # Go to line 2 which is the one with elements numbers
            for _ in range(2):
                f.readline()
            # Save the position
            pos = f.tell()
            # Modify the line with the new one
            old_line = f.readline()
            new_line = f"element vertex {elements_number:>8}\n".encode()
            if len(new_line) != len(old_line):
                raise ValueError("New line is not the same length. Adjust padding.")
            # Change the line inplace
            f.seek(pos)
            f.write(new_line)


    # 12) Boucle de simulation pendant sim_time secondes
    sim_time  = 300.0
    num_ticks = int(sim_time / settings.fixed_delta_seconds)
    print(f"Recording for {sim_time}s → {num_ticks} ticks")

    for tick_counter in range(num_ticks):
        world.tick()
        # Stocker la trajectoire (pour le LiDAR “full_traffic” uniquement)
        loc = vehicle.get_transform().location
        rot = vehicle.get_transform().rotation
        qx, qy, qz, qw = euler_to_quaternion(rot.roll, rot.pitch, rot.yaw)
        ts = world.get_snapshot().timestamp.elapsed_seconds
        vel = vehicle.get_velocity()
        acc = vehicle.get_acceleration()

        update_ply_elements_number(traj_file_ply, tick_counter + 1)
        with open(traj_file_ply, 'ab') as f_ply:
            f_ply.write(struct.pack("<14d", loc.x, loc.y, loc.z, qx, qy, qz, qw, ts, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z))
        with open(traj_file_tum, 'a') as f_tum:
            f_tum.write(f'{ts} {loc.x} {loc.y} {loc.z} {qx} {qy} {qz} {qw}\n')

    print(f"Trajectory PLY saved → {traj_file_ply}")
    print(f"Trajectory TUM saved → {traj_file_tum}")

    # 14) Cleanup : arrêter et détruire tous les LiDAR + acteurs de trafic + véhicule principal
    print("Cleaning up…")
    lidar_no.stop();   lidar_no.destroy()
    lidar_mid.stop();  lidar_mid.destroy()
    lidar_full.stop(); lidar_full.destroy()
    for actor in traffic_vehicles :
        actor.destroy()
    vehicle.destroy()

    world.apply_settings(orig_settings)
    print("Done.")


if __name__ == '__main__':
    main()
