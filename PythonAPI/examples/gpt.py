#!/usr/bin/env python3
import carla, numpy as np, os, math, struct
from plyfile import PlyData, PlyElement

# Configuration
ROT_FREQ    = 20                      # révolutions/s
FIXED_DT    = 0.0005                  # 2 000 Hz
REV_PERIOD  = 1.0 / ROT_FREQ          # 0.05 s
POINTS_P_S  = 640000                  # densité des capteurs

SAVE_DIR    = os.path.expanduser("~/Data_Mathys_full/complet")
os.makedirs(SAVE_DIR, exist_ok=True)

# Buffers
buf_raw     = []
buf_sem     = []
buf_norm    = []

rev_count   = 0
t0          = None

def make_pointcloud(raw_list, sem_list, norm_list):
    """Consolide les listes de frames en tableaux et écrit le PLY."""
    # 1) Empile une seule fois
    arr_raw  = np.vstack([np.frombuffer(f.raw_data, dtype=np.float32).reshape(-1,4)  for f in raw_list])
    arr_sem  = np.vstack([np.frombuffer(f.raw_data, dtype=np.float32).reshape(-1,6)  for f in sem_list])
    arr_norm = np.vstack([np.frombuffer(f.raw_data, dtype=np.float32).reshape(-1,6)  for f in norm_list])
    # 2) Slice commun
    n = min(len(arr_raw), len(arr_sem), len(arr_norm))
    xyz       = arr_raw[:n, :3].astype(np.float32)
    intensity = np.clip((arr_raw[:n, 3] * 255).round(), 0, 255).astype(np.uint16)
    time_st   = np.full((n,), raw_list[0].timestamp, dtype=np.float32)
    semantic  = arr_sem[:n, 3:6].astype(np.float32)
    normal    = arr_norm[:n, 3:6].astype(np.float32)
    # 3) Prépare dtype structuré
    dtype = np.dtype([
        ('x','f4'),('y','f4'),('z','f4'),
        ('intensity','u2'),('time','f4'),
        ('cosine','f4'),('id','u4'),('tag','u4'),
        ('nx','f4'),('ny','f4'),('nz','f4')
    ])
    verts = np.empty(n, dtype=dtype)
    verts['x'], verts['y'], verts['z'] = xyz.T
    verts['intensity'] = intensity
    verts['time']      = time_st
    verts['cosine']    = semantic[:,0]
    verts['id']        = semantic[:,1].astype(np.uint32)
    verts['tag']       = semantic[:,2].astype(np.uint32)
    verts['nx'], verts['ny'], verts['nz'] = normal.T
    # 4) Écriture PLY
    path = os.path.join(SAVE_DIR, f"revo_{rev_count:04d}.ply")
    PlyData([PlyElement.describe(verts, 'vertex')], text=False).write(path)
    print(f"Saved revolution #{rev_count} → {n} points")

def callback_raw(data): buf_raw.append(data)
def callback_sem(data): buf_sem.append(data)
def callback_norm(data): buf_norm.append(data)

def main():
    global t0, rev_count

    client = carla.Client('localhost',2000)
    client.set_timeout(10.0)
    world  = client.get_world()

    # synchrone
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    bp = world.get_blueprint_library()
    # véhicule
    car = bp.find('vehicle.tesla.model3')
    vehicle = world.spawn_actor(car, carla.Transform(carla.Location(x=55.5,y=-57.3,z=0.2)))

    # capteurs
    def make_sensor(id, cb):
        b = bp.find(id)
        b.set_attribute('range','100')
        b.set_attribute('rotation_frequency', str(ROT_FREQ))
        b.set_attribute('channels','32')
        b.set_attribute('points_per_second', str(POINTS_P_S))
        b.set_attribute('sensor_tick', str(FIXED_DT))
        b.set_attribute('upper_fov','15')
        b.set_attribute('lower_fov','-15')
        s = world.spawn_actor(b, carla.Transform(carla.Location(z=2.0)), attach_to=vehicle)
        s.listen(cb)
        return s

    sensor_raw     = make_sensor('sensor.lidar.ray_cast',           callback_raw)
    sensor_sem     = make_sensor('sensor.lidar.ray_cast_semantic',  callback_sem)
    sensor_norm    = make_sensor('sensor.lidar.ray_cast_surface_normals', callback_norm)

    print("Sensors attached, starting recording…")
    # boucle
    while True:
        world.tick()
        # premier timestamp
        if t0 is None and buf_raw:
            t0 = buf_raw[-1].timestamp
        # dès que période atteinte
        if t0 is not None and buf_raw[-1].timestamp - t0 >= REV_PERIOD:
            make_pointcloud(buf_raw, buf_sem, buf_norm)
            rev_count += 1
            # reset
            buf_raw.clear(); buf_sem.clear(); buf_norm.clear()
            t0 = None

if __name__ == '__main__':
    main()
