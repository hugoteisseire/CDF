# Architecture Socket Broadcast LIDAR

## Vue d'ensemble

Le système utilise **deux sockets UNIX** pour diffuser les données LIDAR :

### 1. Socket État/Données LIDAR (`/tmp/robot.sock`)
- **Direction** : LIDAR (client C) → Python (serveur)
- **Rôle** : Le LIDAR envoie son état ET ses données
- **États** : `init`, `ready`, `lost`, `error`
- **Données** : Position, obstacles, etc.

### 2. Socket Broadcast Données (`/tmp/lidar_data.sock`)
- **Direction** : Python (serveur) → STRATEGY, PROG_1, PROG_2, etc. (clients)
- **Rôle** : Redistribue les données LIDAR à tous les clients connectés
- **Architecture** : 1 serveur → N clients (broadcast)

---

## Architecture CORRIGÉE

```
┌─────────────────────────────────────────────────────────────┐
│                  CARTE CONTROL (Python)                     │
│                                                             │
│  ┌───────────────┐         ┌───────────────────────────┐   │
│  │ LidarSocket   │ ──────→ │ LidarDataBroadcastSocket  │   │
│  │ (serveur)     │         │ (serveur)                 │   │
│  └───────┬───────┘         └─────────┬─────────────────┘   │
│          ↑                           ↓                      │
└──────────┼───────────────────────────┼──────────────────────┘
           │                           │
           │ /tmp/robot.sock           │ /tmp/lidar_data.sock
           │ (LIDAR envoie)            │ (Python redistribue)
           ↑                           ↓
    ┌──────────┐           ┌───────────┴──────────┬──────────┐
    │  LIDAR   │           │                      │          │
    │ (CLIENT) │           │    STRATEGY          │  PROG_1  │  PROG_2
    │  test_   │           │    (CLIENT)          │ (CLIENT) │ (CLIENT)
    │  lidar_  │           │                      │          │
    │  socket  │           └──────────────────────┴──────────┴─────────
    └──────────┘
```

**Flux de données :**
1. `test_lidar_socket.c` se connecte à `/tmp/robot.sock` et envoie ses données
2. `LidarSocket` (Python) reçoit les données
3. `LidarSocket` les transfère immédiatement à `LidarDataBroadcastSocket`
4. `LidarDataBroadcastSocket` les diffuse à **tous** les clients connectés
5. `simu_strategy.py` et `simuprog_sel_1.py` reçoivent les données en temps réel

---

## Utilisation

### Côté LIDAR (déjà fait - test_lidar_socket.c)

Votre programme LIDAR continue à fonctionner normalement :
- Se connecte à `/tmp/robot.sock`
- Envoie ses données/état
- Python s'occupe automatiquement de la redistribution

### Côté STRATEGY (déjà intégré)

```python
# simu_strategy.py utilise maintenant LidarReader
lidar_reader = LidarReader()
lidar_reader.start()

while running:
    lidar_data = lidar_reader.get_last_data()
    if lidar_data:
        print(f"LIDAR: {lidar_data}")
```

### Côté PROG_1 (déjà intégré)

```python
# simuprog_sel_1.py utilise le même LidarReader
lidar_reader = LidarReader()
lidar_reader.start()

while running:
    lidar_data = lidar_reader.get_last_data()
    # Utiliser les données...
```

---

## Avantages

✅ **Automatique** : Pas de modification du LIDAR nécessaire  
✅ **Scalable** : Ajoutez autant de programmes lecteurs que nécessaire  
✅ **Découplé** : Le LIDAR ne connaît pas ses lecteurs  
✅ **Robuste** : Si un client crash, les autres continuent  
✅ **Simple** : Un seul socket pour le LIDAR, le reste est transparent  

---

## Fichiers modifiés

- `unix_socket_thread.py` : 
  - `LidarSocket.handle_data()` redistribue via broadcast
  - `LidarDataBroadcastSocket` gère les clients multiples
- `main.py` : Connexion LidarSocket ↔ LidarDataBroadcastSocket
- `simu_strategy.py` : Intégration du LidarReader
- `simuprog_sel_1.py` : Intégration du LidarReader

---

## Test

1. **Démarrer le système** : `python3 main.py`
2. **Appuyer sur BP_INIT** : Lance LIDAR + STRATEGY + PROG_1
3. **Observer** : Les données du LIDAR apparaissent dans STRATEGY et PROG_1

Les trois programmes tourneront en parallèle et recevront tous les mêmes données LIDAR !

