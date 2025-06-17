#!/usr/bin/env python3
import carla
import argparse
import os
import csv
import time

def main(args):
    # Connexion au serveur CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # Récupération ou création d'un véhicule
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.*')[0]
    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[0])
    if vehicle is None:
        actors = world.get_actors().filter('vehicle.*')
        vehicle = actors[0]
    print(f"Véhicule sélectionné (id={vehicle.id})")

    # Prépare le dossier de sortie et le CSV
    os.makedirs(args.output_dir, exist_ok=True)
    csv_path = os.path.join(args.output_dir, 'semantic_lidar.csv')
    with open(csv_path, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            'frame', 'timestamp',
            'x', 'y', 'z',
            'cos_inc_angle',
            'object_idx',
            'object_tag'
        ])

        # Configure le capteur Semantic LIDAR
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('points_per_second', '56000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('upper_fov', '10')
        lidar_bp.set_attribute('lower_fov', '-30')

        transform = carla.Transform(carla.Location(x=0.0, z=2.5))
        lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
        print(f"Semantic LIDAR attaché (id={lidar_sensor.id})")

        def lidar_callback(point_cloud):
            # point_cloud : carla.SemanticLidarMeasurement
            for detection in point_cloud:
                # correction : use `point` et `cos_inc_angle`, `object_idx`, `object_tag` :contentReference[oaicite:0]{index=0}
                loc = detection.point
                writer.writerow([
                    point_cloud.frame,
                    f"{point_cloud.timestamp:.6f}",
                    f"{loc.x:.4f}", f"{loc.y:.4f}", f"{loc.z:.4f}",
                    f"{detection.cos_inc_angle:.4f}",
                    detection.object_idx,
                    detection.object_tag
                ])
            csv_file.flush()

        lidar_sensor.listen(lidar_callback)

        print("Enregistrement en cours... Appuyez sur Ctrl+C pour arrêter.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nArrêt demandé, nettoyage…")
        finally:
            lidar_sensor.stop()
            lidar_sensor.destroy()
            vehicle.destroy()
            print(f"Données sauvegardées dans : {csv_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Attach a Semantic LIDAR sensor to a vehicle and log data to CSV"
    )
    parser.add_argument('--host', default='127.0.0.1',
                        help='IP du serveur CARLA')
    parser.add_argument('--port', default=2000, type=int,
                        help='Port du serveur CARLA')
    parser.add_argument('--output-dir', required=True,
                        help='Dossier où sauvegarder le fichier CSV')
    args = parser.parse_args()
    main(args)
