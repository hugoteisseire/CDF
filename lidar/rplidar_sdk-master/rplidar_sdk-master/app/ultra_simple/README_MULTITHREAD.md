# Architecture Multi-Thread du Programme LIDAR

## 🎯 Objectif
Séparer l'acquisition LIDAR (bloquante) de la lecture IMU et publication socket pour atteindre **50Hz** de mise à jour.

---

## 🧵 Architecture à 2 Threads

### **Thread 1 : Acquisition LIDAR** (`lidar_acquisition_thread`)
- **Tâche** : Acquisition continue des scans LIDAR (bloquante)
- **Fréquence** : Variable (dépend du LIDAR, ~10-15Hz)
- **Données produites** : `shared_scan` (protégé par `scan_mutex`)
- **État** : Met à jour `shared_current_state` (init/error)

```
┌─────────────────────────────────┐
│  Thread LIDAR (bloquant)        │
│  ─────────────────────────      │
│  while(running) {                │
│    grabAndUpdateScan()   ◄──────┼─── BLOQUANT (attend données LIDAR)
│    {lock}                        │
│      shared_scan = local_scan   │
│      new_scan_available = true  │
│    {unlock}                      │
│  }                               │
└─────────────────────────────────┘
```

---

### **Thread 2 : IMU + Calculs + Publication** (`imu_compute_publish_thread`)
- **Tâche** : Lecture IMU, calculs de position, publication socket
- **Fréquence** : **50Hz fixe** (20ms par itération)
- **Données consommées** : `shared_scan` (lecture)
- **Données produites** : `shared_posImu`, `shared_poslidar`, `shared_posennemy` (protégés par `position_mutex`)

```
┌──────────────────────────────────────────────────────────┐
│  Thread IMU+Publication (50Hz = 20ms/loop)               │
│  ──────────────────────────────────────                  │
│  while(running) {                                        │
│    auto start = steady_clock::now();                     │
│                                                          │
│    1) Lecture IMU (rapide, non-bloquant)                │
│       imu.readPose(pose)                                 │
│                                                          │
│    2) Copie du dernier scan LIDAR                       │
│       {lock scan_mutex}                                  │
│         local_scan = shared_scan                         │
│       {unlock}                                           │
│                                                          │
│    3) Calculs (piliers, pose, adversaire)               │
│       trackPoints(), computePose(), detectOpponent()     │
│                                                          │
│    4) Mise à jour positions partagées                   │
│       {lock position_mutex}                              │
│         shared_posImu = posImu                           │
│         shared_poslidar = poslidar                       │
│         shared_posennemy = posennemy                     │
│       {unlock}                                           │
│                                                          │
│    5) Publication socket (état + 20 floats)             │
│       write(client_sock, state, ...)                     │
│       write(client_sock, data[20], ...)                  │
│                                                          │
│    6) Maintenir 50Hz                                     │
│       sleep_until(start + 20ms)                          │
│  }                                                       │
└──────────────────────────────────────────────────────────┘
```

---

## 🔒 Synchronisation (Points Critiques)

### **Variables Partagées**

| Variable | Type | Protection | Accès |
|----------|------|------------|-------|
| `shared_scan` | `std::vector<float>` | `scan_mutex` | Thread LIDAR (W), Thread IMU (R) |
| `shared_posImu` | `position` | `position_mutex` | Thread IMU (W), Socket (R) |
| `shared_poslidar` | `position` | `position_mutex` | Thread IMU (W), Socket (R) |
| `shared_posennemy` | `position` | `position_mutex` | Thread IMU (W), Socket (R) |
| `shared_trackedPoints` | `std::vector<TrackResult>` | `position_mutex` | Thread IMU (W), Socket (R) |
| `new_scan_available` | `std::atomic<bool>` | Atomic | Thread LIDAR (W), Thread IMU (R) |
| `running` | `std::atomic<bool>` | Atomic | Main (W), Threads (R) |
| `shared_current_state` | `const char*` | Atomic pointer | Thread LIDAR/IMU (W), Socket (R) |

### **Mutex utilisés**
1. **`scan_mutex`** : Protège le scan LIDAR partagé
2. **`position_mutex`** : Protège toutes les positions calculées

### **Pattern utilisé : Lock + Copie locale**
```cpp
// ✅ BON : Lock court, copie locale
position local_pos;
{
    std::lock_guard<std::mutex> lock(position_mutex);
    local_pos = shared_posImu;  // Copie rapide
}
// Utiliser local_pos sans lock

// ❌ MAUVAIS : Lock long
{
    std::lock_guard<std::mutex> lock(position_mutex);
    // Calculs lourds ou I/O ici = DEADLOCK possible
}
```

---

## 📊 Diagramme de Flux de Données

```
┌─────────────┐
│   LIDAR     │
│  Hardware   │
└──────┬──────┘
       │ (bloquant)
       ▼
┌────────────────────┐
│ Thread LIDAR       │
│ grabAndUpdateScan()│
└─────────┬──────────┘
          │ {scan_mutex}
          ▼
    ┌──────────────┐
    │ shared_scan  │
    └──────┬───────┘
           │ (lecture)
           ▼
    ┌────────────────────────┐
    │ Thread IMU+Publication │
    │ - Lecture IMU          │
    │ - Calcul positions     │
    │ - Détection adversaire │
    └──────┬─────────────────┘
           │ {position_mutex}
           ▼
    ┌──────────────────────┐
    │ shared_posImu        │
    │ shared_poslidar      │
    │ shared_posennemy     │
    └──────┬───────────────┘
           │
           ▼
    ┌────────────────┐
    │ Socket Python  │ (50Hz)
    └────────────────┘
```

---

## ⚡ Performance

### **Avant (Single Thread)**
- Fréquence : **~10-15Hz** (limité par `grabAndUpdateScan` bloquant)
- Problème : IMU et socket bloqués pendant acquisition LIDAR

### **Après (Multi-Thread)**
- Thread LIDAR : **~10-15Hz** (limité hardware)
- Thread IMU+Pub : **50Hz fixe** (20ms/loop)
- Gain : **3-5x** sur la fréquence IMU/Publication

---

## 🚀 Utilisation

### **Compilation**
```bash
make
```

### **Lancement**
```bash
# Mode production (silencieux)
./ultra_simple 0

# Mode debug (verbeux)
./ultra_simple 0 debug

# Mode debug auto (team 3)
./ultra_simple 3
```

### **Arrêt propre**
- `Ctrl+C` → Met `running = false` → Threads s'arrêtent proprement

---

## ⚠️ Points d'attention

1. **Ordre de démarrage** : Serveur Python → Programme LIDAR
2. **Socket** : Connexion CLIENT avec retry (10 tentatives)
3. **Arrêt** : Toujours `Ctrl+C` (jamais `kill -9`)
4. **Debug** : Utiliser `DEBUG_PRINT = true` pour voir l'activité des threads
5. **I2C** : IMU sur `/dev/i2c-4` à `0x17` (vérifier si autre config)

---

## 📝 Compilation flags

Vérifier que le Makefile compile avec `-std=c++11` minimum et `-pthread` :

```makefile
CXXFLAGS += -std=c++17 -pthread
```

---

## 🐛 Debugging

### **Vérifier l'activité des threads**
```bash
./ultra_simple 3  # Mode debug auto

# Sortie attendue :
# 🚀 Démarrage du système multi-thread...
# 🔄 Thread LIDAR démarré
# 🔄 Thread IMU+Publication démarré
# [logs de fonctionnement...]
# ^C
# ⏳ Arrêt des threads...
# 🛑 Thread LIDAR arrêté
# 🛑 Thread IMU+Publication arrêté
# ✅ Threads arrêtés
```

### **Vérifier deadlocks**
Si le programme freeze, vérifier :
- Ordre d'acquisition des mutex (toujours le même)
- Durée de lock (doit être courte)
- Pas d'I/O dans les sections critiques

---

## 📚 Références

- **std::thread** : https://en.cppreference.com/w/cpp/thread/thread
- **std::mutex** : https://en.cppreference.com/w/cpp/thread/mutex
- **std::atomic** : https://en.cppreference.com/w/cpp/atomic/atomic
- **std::lock_guard** : https://en.cppreference.com/w/cpp/thread/lock_guard
