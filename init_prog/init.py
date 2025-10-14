import subprocess
import json
import os
from time import sleep

# 1. Initialisation des données (exemple)
init_data = {
    "robot": {
        "x": 10.0,      # float
        "y": 20.0,      # float
        "angle": 45.0   # float
    },
    "equipe": False     # booléen (par défaut)
}

# 2. Écriture dans un fichier JSON (pour le programme C)
with open("init_data.json", "w") as f:
    json.dump(init_data, f, indent=4)

# 3. Lancement du programme C en parallèle
#    (le script Python continue son exécution)
# a partir du curdirectoire courant créer path


current_dir = os.getcwd()
path_to_c_program = os.path.join(
    current_dir, "../lidar/rplidar_sdk-master/rplidar_sdk-master/output/Linux/Release/ultra_simple")
proc = subprocess.Popen([path_to_c_program])

# 4. Le script Python peut continuer à faire autre chose ici
print("Programme C lancé en parallèle. Le script Python continue...")

sleep(3)
proc.kill()
