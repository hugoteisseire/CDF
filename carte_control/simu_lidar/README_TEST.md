# Test du système LIDAR via socket UNIX

## Architecture

**Python = SERVEUR** (crée le socket et attend les connexions)  
**Programme C = CLIENT** (se connecte au socket Python)

Cette architecture permet:
- ✅ Python démarre en premier et crée le socket
- ✅ Les programmes C peuvent se connecter/déconnecter
- ✅ Redémarrage du programme C sans redémarrer Python
- ✅ Python contrôle le cycle de vie complet

## Compilation du simulateur LIDAR

Sur le Raspberry Pi :

```bash
cd /home/raspi/Desktop/CDF/lidar
make
```

Cela va compiler `test_lidar_socket.c` et créer l'exécutable `test_lidar_socket`.

## Test manuel

### 1. Démarrer le serveur Python

```bash
cd /home/raspi/Desktop/CDF/carte_control
python3 main.py
```

Vous devriez voir :
```
🎯 Serveur socket LIDAR démarré
En attente de connexion sur /tmp/robot.sock...
```

### 2. Dans un autre terminal, démarrer le simulateur C (client)

```bash
cd /home/raspi/Desktop/CDF/lidar
./test_lidar_socket
```

Vous verrez :
```
🔌 Simulateur LIDAR (CLIENT)
Connexion au serveur Python sur /tmp/robot.sock...
✅ Connecté au serveur Python!
```

## Test automatique (recommandé)

Le programme `main.py` démarre automatiquement le simulateur LIDAR :

```bash
cd /home/raspi/Desktop/CDF/carte_control
python3 main.py
```

Le programme va :
1. ✅ Créer le serveur socket `/tmp/robot.sock`
2. ✅ Démarrer le simulateur LIDAR C qui se connecte au socket
3. ✅ Afficher les états reçus toutes les secondes
4. ✅ Arrêter proprement tous les processus avec Ctrl+C

## Format des données

Le simulateur envoie une **chaîne de caractères** terminée par `\n` toutes les 2 secondes.

### États possibles :

| État | Description | Exemple d'usage |
|------|-------------|-----------------|
| `init` | Initialisation du LIDAR | Au démarrage |
| `ready` | LIDAR prêt, en attente | Avant le match |
| `running` | Tracking actif | Pendant le match |
| `lost` | Position perdue | Problème de localisation |
| `error` | Erreur matérielle | Dysfonctionnement |

### Séquence simulée :

Le simulateur cycle à travers ces états :
```
init → ready → running → lost → ready → running → (boucle)
```

## Exemple de sortie

### Programme C (serveur) :
```
🎯 Simulateur LIDAR démarré sur /tmp/robot.sock
En attente de connexion...
✅ Client connecté!
Envoi des états simulés...

[0000] État envoyé: init
[0001] État envoyé: ready
[0002] État envoyé: running
[0003] État envoyé: lost
```

### Programme Python (client) :
```
==================================================
✅ État LIDAR: READY
🎛️  Switches: {'team': False, 'strat': 0, 'test': 0}
✅ Tracking actif
```

## Nettoyage

Pour supprimer le socket et l'exécutable :

```bash
cd /home/raspi/Desktop/CDF/lidar
make clean
```

## Intégration avec le vrai programme LIDAR

Dans votre vrai programme LIDAR C++, ajoutez simplement :

```c
#include <sys/socket.h>
#include <sys/un.h>

// Au démarrage
int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
struct sockaddr_un addr;
addr.sun_family = AF_UNIX;
strcpy(addr.sun_path, "/tmp/robot.sock");

// Connexion au serveur Python (avec retry)
while (connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    sleep(1);  // Réessayer jusqu'à ce que Python soit prêt
}

// Dans votre boucle principale
const char *state = "running\n";  // ou "init", "ready", "lost", "error"
write(sock_fd, state, strlen(state));
```

Modifiez ensuite `main.py` :

```python
# Ancien (simulateur)
lidar_executable = '/home/raspi/Desktop/CDF/lidar/test_lidar_socket'

# Nouveau (vrai programme)
lidar_executable = '/home/raspi/Desktop/CDF/lidar/rplidar_sdk-master/rplidar_sdk-master/app/ultra_simple/ultra_simple'
```

## Avantages de cette architecture

- 🔄 Le programme C peut crasher/redémarrer sans affecter Python
- 🎯 Python garde toujours le contrôle du socket
- 🔌 Facile d'ajouter d'autres clients (IMU, Vision, etc.)
- 🧹 Nettoyage automatique du socket à l'arrêt de Python
