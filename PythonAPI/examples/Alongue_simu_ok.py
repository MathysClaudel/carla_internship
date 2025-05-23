#!/usr/bin/env python3

from plyfile import PlyData, PlyElement
import numpy as np
import carla
import os, glob, sys, time
from queue import Queue
import random
import math
import struct




# def add_ply_with_time(path, points, timestamp):
#     with open(path, 'a') as f:
#         for x, y, z, i in points:
#             intensite = int(round(i * 255))
#             f.write(f"{x:.6f} {y:.6f} {z:.6f} {intensite:d} {timestamp:.6f}\n")

# def insert_ply(path):

#     with open(path, 'r') as f:
#         nb_points = sum(1 for _ in f)

#     header = [
#         "ply\n",
#         "format ascii 1.0\n",
#         f"element vertex {nb_points}\n",
#         "property float x\n",
#         "property float y\n",
#         "property float z\n",
#         "property ushort intensity\n",
#         "property float time\n",
#         "end_header\n"
#     ]

#     with open(path, 'r') as f:
#         data_lines = f.readlines()

#     with open(path, 'w') as f:
#         f.writelines(header)
#         f.writelines(data_lines)



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
        PlyData([el], text=False).write(path) #mettre true pour avoir en sscii


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
    spawn_point = carla.Transform(carla.Location(x=55.5, y=-57.3, z=0.2))
    vehicle     = world.spawn_actor(car_bp, spawn_point)

    #spawn_point = random.choice(world.get_map().get_spawn_points())
    #vehicle  = world.spawn_actor(car_bp, spawn_point)



    # 6) Spawn du LiDAR “ring-par-tick”
    rotation_frequency = 20 #20
    sensor_tick = 0.0005 #DOIT ETRE EGAL AU DELTA_TIME_SIMULATE (LOGIQUE)


    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range',              '100')
    lidar_bp.set_attribute('rotation_frequency', str(rotation_frequency))     # 10 révolutions/s
    lidar_bp.set_attribute('channels',           '32')     # 32 
    lidar_bp.set_attribute('points_per_second',  '640000') # densité
    lidar_bp.set_attribute('noise_stddev',       '0.02') #2cm
    lidar_bp.set_attribute('sensor_tick',         str(sensor_tick))  # 1 ms simulé = 1 tick
    lidar_bp.set_attribute('upper_fov',           '15')
    lidar_bp.set_attribute('lower_fov',           '-15')
    lidar_bp.set_attribute('enable_ego_motion',   'true')

    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_queue = Queue()
    lidar.listen(lambda data: lidar_queue.put(data))
    print("LiDAR attached and listening…")

    #########################################################

    lidar2_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar2_bp.set_attribute('range',              '100')
    lidar2_bp.set_attribute('rotation_frequency', str(rotation_frequency))     # 10 révolutions/s
    lidar2_bp.set_attribute('channels',           '32')     # 32 
    lidar2_bp.set_attribute('points_per_second',  '640000') # densité
    lidar2_bp.set_attribute('noise_stddev',       '0.02') #2cm
    lidar2_bp.set_attribute('sensor_tick',        str(sensor_tick))  # 1 ms simulé = 1 tick
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
    print("LiDAR attached and listening…")



    # 5) Warm-up 1 s pour que la physique se stabilise
    warmup_time  = 0.5
    warmup_ticks = int(warmup_time / settings.fixed_delta_seconds)
    for _ in range(warmup_ticks):
        world.tick()


    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    vehicle.set_autopilot(True, tm.get_port())


    print(f"Spawned vehicle at {spawn_point.location}")


    # 7) Enregistrement pendant 8 s simulées
    sim_time  = 300.0
    num_ticks = int(sim_time / settings.fixed_delta_seconds)
    save_dir1  = os.path.expanduser("~/Data_Mathys_full/22mai/1") #achanger
    os.makedirs(save_dir1, exist_ok=True)
    template1  = os.path.join(save_dir1, "trame_%08d.ply")
    save_dir2  = os.path.expanduser("~/Data_Mathys_full/22mai/onsenfout")
    os.makedirs(save_dir2, exist_ok=True)
    template2  = os.path.join(save_dir2, "trame_%08d.ply")
    print(f"Recording for {sim_time}s → {num_ticks} ticks")
    k=int(1/(sensor_tick * rotation_frequency))
    n=k-1
    rev_counter=0

    pc=PointCloud()
    pc2=PointCloud()

    for _ in range(num_ticks):
        world.tick()
        # on vide la file au besoin (normalement 1 frame LiDAR par tick)
        while not lidar_queue.empty():
            meas = lidar_queue.get()
            meas2 = lidar2_queue.get()
            arr  = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1,4)
            arr2  = np.frombuffer(meas2.raw_data, dtype=np.float32).reshape(-1,4)
            ts   = meas.timestamp
            ts2   = meas.timestamp
            n=n+1
            vel = lidar.get_velocity()
            acc = lidar.get_acceleration()
            # if n%k == 0 :
            #     filename = template % meas.frame
            #     add_ply_with_time(filename,arr, ts)
            # else :
            #     if n%k == k-1 :
            #         add_ply_with_time(filename,arr, ts)
            #         insert_ply(filename)
            #     else :
            #         add_ply_with_time(filename,arr, ts)

            if n%k == k-1 :
                pc.add_points(arr[:,:3], arr[:,3], ts)
                filename = template1 % rev_counter
                rev_counter+= 1
                pc.save(filename)
                print(f"Saved full revolution #{rev_counter}")
                #rev_counter += 1 
                
                pc = PointCloud()
                pc2.add_points(arr2[:,:3], arr2[:,3], ts2)
                filename = template2 % rev_counter
                pc2.save(filename)
                print(f"Saved full revolution #{rev_counter}")
                #rev_counter += 1 
                pc2 = PointCloud()


                transform = meas.transform
                location  = transform.location
                rotation  = transform.rotation
                qx, qy, qz, qw = euler_to_quaternion(rotation.roll, rotation.pitch, rotation.yaw)
                trajectory.append((location.x, location.y, location.z, qx, qy, qz, qw, ts, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z))
            else :
                pc.add_points(arr[:,:3], arr[:,3], ts)
                pc2.add_points(arr2[:,:3], arr2[:,3], ts2)
                transform = meas.transform
                location  = transform.location
                rotation  = transform.rotation
                qx, qy, qz, qw = euler_to_quaternion(rotation.roll, rotation.pitch, rotation.yaw)
                trajectory.append((location.x, location.y, location.z, qx, qy, qz, qw, ts, vel.x, vel.y, vel.z, acc.x, acc.y, acc.z))
            


    with open(os.path.join(save_dir1, "trajectory.ply"), "wb") as f:  #achanger
        f.write(b"ply\n")
        f.write(b"format binary_little_endian 1.0\n")
        f.write(f"element vertex {len(trajectory)}\n".encode())
        f.write(b"property double x\n")
        f.write(b"property double y\n")
        f.write(b"property double z\n")
        f.write(b"property double q_x\n")
        f.write(b"property double q_y\n")
        f.write(b"property double q_z\n")
        f.write(b"property double q_w\n")
        f.write(b"property double timestamp\n")
        f.write(b"property double vel_x\n")
        f.write(b"property double vel_y\n")
        f.write(b"property double vel_z\n")
        f.write(b"property double acc_x\n")
        f.write(b"property double acc_y\n")
        f.write(b"property double acc_z\n")
        f.write(b"end_header\n")

        for pose in trajectory:
            f.write(struct.pack("<14d", *pose))


    # 8) Cleanup
    print("Cleaning up…")
    lidar.stop()
    lidar.destroy()
    vehicle.destroy()
    world.apply_settings(orig_settings)
    print("Done.")

if __name__ == "__main__":
    main()
