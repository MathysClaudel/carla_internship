# -*- coding: utf-8 -*-
import glob
import os
import sys
import random
import math
import struct

# (Éventuel code pour ajouter le CARLA egg dans sys.path)...

import carla

# ───────────── Monkey‐patch pour injecter set_ignored_actor_ids ─────────────
SensorClass = carla.libcarla.ServerSideSensor
if not hasattr(SensorClass, 'set_ignored_actor_ids'):
    def set_ignored_actor_ids(self, actor_ids):
        return self.call('SetIgnoredActorIDs', actor_ids)
    setattr(SensorClass, 'set_ignored_actor_ids', set_ignored_actor_ids)
    print("→ set_ignored_actor_ids injecté dans ServerSideSensor")
else:
    print("→ set_ignored_actor_ids déjà présent dans ServerSideSensor")
# ─────────────────────────────────────────────────────────────────────────────

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world = client.get_world()

    # (Configuration synchrone, traffic manager, blueprint_lib, etc.)

    # Exemple de spawn de véhicules/traffic:
    all_vehicle_bps = world.get_blueprint_library().filter('vehicle.*')
    traffic_vehicles = []
    for pt in spawn_points[:10]:
        veh = world.try_spawn_actor(random.choice(all_vehicle_bps), pt)
        if veh:
            traffic_vehicles.append(veh)

    # On récupère les IDs à ignorer
    ignore_all_ids  = [v.id for v in traffic_vehicles]
    ignore_half_ids = [v.id for v in traffic_vehicles[:len(traffic_vehicles)//2]]
    ignore_none_ids = []

    # Blueprint LiDAR complet
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_complete')
    for k, v in {
        'range': '100',
        'rotation_frequency': '20',
        'channels': '32',
        'points_per_second': '640000',
        'noise_stddev': '0.02',
        'sensor_tick': str(0.0005),
        'upper_fov': '15',
        'lower_fov': '-15',
        'enable_ego_motion': 'true'
    }.items():
        lidar_bp.set_attribute(k, v)

    # Spawn du LiDAR “no_traffic”
    lidar_no = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    # On appelle la méthode injectée
    lidar_no.set_ignored_actor_ids(ignore_all_ids)
    lidar_no.listen(callback_no)

    # Spawn du LiDAR “mid_traffic”
    lidar_mid = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_mid.set_ignored_actor_ids(ignore_half_ids)
    lidar_mid.listen(callback_mid)

    # Spawn du LiDAR “full_traffic”
    lidar_full = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.0)),
        attach_to=vehicle
    )
    lidar_full.set_ignored_actor_ids(ignore_none_ids)
    lidar_full.listen(callback_full)

    # Suite du script : tick(), enregistrement, cleanup, etc.

if __name__ == '__main__':
    main()
