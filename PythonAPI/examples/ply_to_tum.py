#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convertit un PLY binaire (format little endian) contenant une trajectoire
avec ce header :

  ply
  format binary_little_endian 1.0
  element vertex N
    property double x
    property double y
    property double z
    property double q_w
    property double q_x
    property double q_y
    property double q_z
    property double timestamp
    property int    indices
    property float  std_dev_x
    property float  std_dev_y
    property float  std_dev_z
    property float  std_dev_norm
  end_header

en un fichier TUM (.tum) au format :

  timestamp  x  y  z  q_x  q_y  q_z  q_w

Usage :
    python ply_to_tum.py <input_traj.ply> <output_traj.tum>
"""

import sys
import os
import numpy as np

try:
    from plyfile import PlyData
except ImportError:
    print("Erreur : la bibliothèque 'plyfile' n'est pas installée.")
    print("         Installez-la via : pip install plyfile")
    sys.exit(1)


def ply_to_tum(in_ply: str, out_tum: str):
    # Vérification que le fichier PLY existe
    if not os.path.isfile(in_ply):
        print(f"Erreur : le fichier '{in_ply}' n'existe pas.")
        sys.exit(1)

    # Lecture du PLY (binaire little endian ou ASCII)
    try:
        plydata = PlyData.read(in_ply)
    except Exception as e:
        print(f"Erreur lors de la lecture du PLY '{in_ply}' : {e}")
        sys.exit(1)

    # Trouver l'élément 'vertex' (sans utiliser element_names)
    vertex = None
    for elem in plydata.elements:
        if elem.name == 'vertex':
            vertex = elem
            break

    if vertex is None:
        print("Erreur : le PLY ne contient pas d'élément 'vertex'.")
        sys.exit(1)

    props = vertex.data.dtype.names  # noms de toutes les propriétés sous 'vertex'

    # Champs obligatoires pour générer le TUM
    required = ['x', 'y', 'z', 'q_w', 'q_x', 'q_y', 'q_z', 'timestamp']
    for champ in required:
        if champ not in props:
            print(f"Erreur : la propriété '{champ}' est manquante dans le PLY.")
            sys.exit(1)

    # Extraction des tableaux numpy (float64)
    x_arr  = np.array(vertex.data['x'],       dtype=np.float64)
    y_arr  = np.array(vertex.data['y'],       dtype=np.float64)
    z_arr  = np.array(vertex.data['z'],       dtype=np.float64)
    qw_arr = np.array(vertex.data['q_w'],     dtype=np.float64)
    qx_arr = np.array(vertex.data['q_x'],     dtype=np.float64)
    qy_arr = np.array(vertex.data['q_y'],     dtype=np.float64)
    qz_arr = np.array(vertex.data['q_z'],     dtype=np.float64)
    ts_arr = np.array(vertex.data['timestamp'], dtype=np.float64)

    # Vérification que tous les tableaux ont la même taille
    n = ts_arr.shape[0]
    if not (
        x_arr.shape[0] == y_arr.shape[0] == z_arr.shape[0] ==
        qw_arr.shape[0] == qx_arr.shape[0] == qy_arr.shape[0] ==
        qz_arr.shape[0] == ts_arr.shape[0] == n
    ):
        print("Erreur : les tableaux de champs n’ont pas tous la même longueur.")
        sys.exit(1)

    # Tri par timestamp (ordre croissant)
    indices_tries = np.argsort(ts_arr)

    # Écriture du fichier .tum
    try:
        with open(out_tum, 'w') as f:
            for i in indices_tries:
                # Format TUM attendu par EVO :
                #   timestamp  x  y  z  q_x  q_y  q_z  q_w
                f.write(
                    f"{ts_arr[i]:.6f} "
                    f"{x_arr[i]:.6f} {y_arr[i]:.6f} {z_arr[i]:.6f} "
                    f"{qx_arr[i]:.6f} {qy_arr[i]:.6f} {qz_arr[i]:.6f} {qw_arr[i]:.6f}\n"
                )
    except Exception as e:
        print(f"Erreur lors de l’écriture du fichier '{out_tum}' : {e}")
        sys.exit(1)

    print(f"Conversion réussie : '{out_tum}' généré ({n} poses, triées).")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage : python ply_to_tum.py <input_traj.ply> <output_traj.tum>")
        sys.exit(1)

    chemin_ply = sys.argv[1]
    chemin_tum = sys.argv[2]
    ply_to_tum(chemin_ply, chemin_tum)
