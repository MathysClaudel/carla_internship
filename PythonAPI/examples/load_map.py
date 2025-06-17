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

def main():
    # connexion au serveur CARLA (doit être lancé au préalable sur localhost:2000)
    client = carla.Client('localhost', 2000)
    client.set_timeout(600.0)

    # forcer le chargement de Town_N
    sim_world = client.load_world('Town03') #changer le numéro de la town pour forcer la map
    print(f"Map chargée : {sim_world.get_map().name}")

if __name__ == '__main__':
    main()