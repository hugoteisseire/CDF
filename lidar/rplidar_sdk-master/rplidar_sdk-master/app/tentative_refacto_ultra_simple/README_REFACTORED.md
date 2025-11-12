# 🤖 LIDAR Localization - Code Refactorisé

## 📋 Vue d'ensemble

Ce dossier contient la version **refactorisée** du code de localisation LIDAR pour la Coupe de France de Robotique.

### Améliorations principales

✅ **Configuration centralisée** : `robot_config.h` regroupe toutes les constantes  
✅ **Enum Team** : Typage fort pour les équipes (TEAM0, TEAM1, DEBUG)  
✅ **Code modulaire** : Fonctions décomposées et testables  
✅ **Structures d'état** : Encapsulation des variables globales  
✅ **Documentation Doxygen** : Toutes les fonctions documentées  
✅ **Gestion d'erreurs** : Messages clairs et logs emoji  
✅ **Nommage explicite** : Unités dans les noms de variables (`_mm`, `_rad`)

---

## 📁 Structure des fichiers

```
ultra_simple/
├── robot_config.h              # ⚙️ Configuration centralisée (NOUVEAU)
├── lidar_lib_refactored.h      # 📚 Header refactorisé (NOUVEAU)
├── lidar_lib_refactored.cpp    # 📚 Implémentation refactorée (NOUVEAU)
├── main_refactored.cpp         # 🚀 Main refactorisé (NOUVEAU)
│
├── lidar_lib.h                 # 📚 Version originale (CONSERVÉE)
├── lidar_lib.cpp               # 📚 Version originale (CONSERVÉE)
├── main.cpp                    # 🚀 Version originale (CONSERVÉE)
├── ImuOTOS.h/cpp               # IMU OTOS (inchangé)
└── Makefile                    # Build system
```

---

## 🔧 Configuration (`robot_config.h`)

### Équipes et positions

```cpp
enum class Team {
    TEAM0 = 0,  // Côté droit (x=2200mm)
    TEAM1 = 1,  // Côté gauche (x=800mm)
    DEBUG = 3   // Centre table (x=750mm)
};
```

### Constantes principales

| Constante | Valeur | Description |
|-----------|--------|-------------|
| `LIDAR_RESOLUTION_DEG` | 0.25° | Résolution angulaire |
| `LIDAR_NUM_ANGLES` | 1440 | Nombre d'angles (360°/0.25°) |
| `PILLAR_RADIUS_MM` | 50 mm | Rayon d'un pilier |
| `PILLAR_POSITION_TOLERANCE_MM` | 450 mm | Tolérance de détection |
| `IMU_SMOOTHING_WINDOW_SIZE` | 5 | Taille fenêtre de lissage |

### Positions des piliers

Les positions sont **automatiquement chargées** selon l'équipe :

```cpp
const std::vector<Position> PILLARS_TEAM0 = {
    {2950.0f, 50.0f, 0.0f},     // Coin bas-droit
    {2950.0f, 1950.0f, 0.0f},   // Coin haut-droit
    {50.0f, 1000.0f, 0.0f}      // Milieu gauche
};
```

---

## 🏗️ Architecture refactorée

### Structure `RobotState`

Encapsule toutes les positions et vitesses :

```cpp
struct RobotState {
    Position posRobot;      // Position fusionnée IMU+LIDAR
    Position posIMU;        // Position IMU brute
    Position posLIDAR;      // Position LIDAR calculée
    ImuPose velocity;       // Vitesse du robot
    std::deque<Position> posHistory;  // Historique pour lissage
};
```

### Structure `LocalizationState`

Gère le tracking des piliers :

```cpp
struct LocalizationState {
    std::vector<Position> pillarPositions;
    std::vector<TrackResult> trackedPoints;
    int consecutiveDetections;
};
```

### Structure `SocketState`

Encapsule la communication socket :

```cpp
struct SocketState {
    int serverSock;
    int clientSock;
    const char* socketPath;
};
```

---

## 🔄 Flux d'exécution

```
main()
  ├─> Initialisation config (Team, signaux)
  ├─> Initialisation socket
  ├─> Initialisation localisation (piliers, position initiale)
  ├─> Initialisation IMU
  ├─> Initialisation LIDAR
  └─> mainLoop()
       ├─> sendDataToClient()           // Envoi socket
       ├─> updateIMUPosition()          // Lecture IMU
       ├─> updateVelocity()             // Lecture vitesse
       ├─> processScan()                // Acquisition + tracking
       │    ├─> grabAndUpdateScan()
       │    └─> trackPillars()
       ├─> computeAndUpdatePose()       // Calcul pose LIDAR
       ├─> fusePoses()                  // Fusion IMU/LIDAR
       │    └─> shouldFuseWithIMU()
       └─> displayStatus()              // Affichage
```

---

## 📡 Fonctions principales

### Initialisation

```cpp
std::vector<Position> initPillars(Team team);
std::vector<TrackResult> initTrackedPoints(const Position& robotPos, 
                                            const std::vector<Position>& pillars);
bool initIMU(ImuOTOS& imu);
ILidarDriver* initLidar(int argc, const char* argv[], ...);
```

### Traitement

```cpp
bool grabAndUpdateScan(LidarScan& scan, ILidarDriver* drv);
void trackPillars(const LidarScan& scan, 
                  std::vector<TrackResult>& trackedPoints,
                  const Position& robotPos,
                  const std::vector<Position>& pillars);
Position computeRobotPose(const std::vector<TrackResult>& measurements,
                          const std::vector<Position>& pillars);
```

### Utilitaires

```cpp
float findPillarCenter(const LidarScan& scan, float angle, float distance);
ImuPose positionToImuPose(const Position& pos);
Position polarToCartesian(const TrackResult& measurement);
```

---

## 🚀 Compilation et exécution

### Makefile modifié

Ajouter au Makefile :

```makefile
ultra_simple_refactored: main_refactored.o lidar_lib_refactored.o ImuOTOS.o
	$(CXX) -o $@ $^ $(LDFLAGS)

main_refactored.o: main_refactored.cpp robot_config.h lidar_lib_refactored.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

lidar_lib_refactored.o: lidar_lib_refactored.cpp robot_config.h lidar_lib_refactored.h
	$(CXX) $(CXXFLAGS) -c $< -o $@
```

### Compilation

```bash
make ultra_simple_refactored
```

### Exécution

```bash
# Équipe 0 (côté droit)
./ultra_simple_refactored 0 --channel --serial /dev/serial0 256000

# Équipe 1 (côté gauche)
./ultra_simple_refactored 1 --channel --serial /dev/serial0 256000

# Mode debug (centre)
./ultra_simple_refactored 3 --channel --serial /dev/serial0 256000
```

---

## 🔍 Différences clés avec l'original

| Aspect | Original | Refactorisé |
|--------|----------|-------------|
| **Constantes** | Dispersées dans .cpp | Centralisées dans `robot_config.h` |
| **Type équipe** | `int team` | `enum class Team` |
| **Variables globales** | `server_sock`, `client_sock` | Encapsulées dans `SocketState` |
| **mainLoop** | 180 lignes monolithique | Décomposé en 8 fonctions |
| **Nommage** | `max_dist`, `da`, `dd` | `TRACKING_MAX_DISTANCE_MM`, `angle_diff`, `dist_diff` |
| **Erreurs** | `std::cerr` simple | Messages emoji + contexte |
| **Documentation** | Absente | Doxygen complet |
| **Testabilité** | Impossible (couplage fort) | Testable (fonctions pures) |

---

## 📊 Avantages

### 1. Maintenabilité
- Modification facile des constantes (un seul fichier)
- Fonctions courtes et focalisées
- Responsabilités claires

### 2. Robustesse
- Gestion d'erreurs systématique
- Validation des données
- Messages d'erreur informatifs

### 3. Testabilité
- Fonctions pures sans état global
- Structures d'état passées en paramètres
- Mock facile pour les tests

### 4. Lisibilité
- Documentation Doxygen
- Nommage explicite avec unités
- Structure logique

---

## 🔮 Évolutions futures possibles

### P1 - Critique
- [ ] Chargement config depuis JSON au runtime
- [ ] Tests unitaires (Google Test)
- [ ] Retry logic pour LIDAR/IMU

### P2 - Important
- [ ] Logging structuré (fichier + console)
- [ ] Watchdog pour détection freeze
- [ ] Statistiques temps réel (FPS, latence)

### P3 - Nice to have
- [ ] Interface web pour monitoring
- [ ] Enregistrement scans pour replay
- [ ] Calibration automatique piliers

---

## 📝 Notes de migration

### Pour basculer sur la version refactorée :

1. **Tester en parallèle** : Les deux versions coexistent
2. **Valider les résultats** : Comparer les poses calculées
3. **Mise à jour Makefile** : Compiler `ultra_simple_refactored`
4. **Scripts de lancement** : Remplacer `ultra_simple` par `ultra_simple_refactored`

### Compatibilité

✅ **Format socket identique** : Le protocole de communication n'a pas changé  
✅ **Arguments compatibles** : `./ultra_simple_refactored <team> --channel ...`  
✅ **IMU/LIDAR** : Mêmes drivers et API

---

## 🐛 Debug et logs

Les logs utilisent des **emoji** pour faciliter la lecture :

| Emoji | Signification |
|-------|--------------|
| 🎯 | Initialisation piliers |
| 📡 | Acquisition scan |
| ✅ | Succès |
| ❌ | Erreur critique |
| ⚠️ | Warning |
| 📍 | Position |
| 🚀 | Démarrage/Vitesse |
| 🛑 | Arrêt |

---

## 👥 Auteurs

**Version originale** : Hugo Teisseire (NBcra)  
**Refactorisation** : GitHub Copilot + Hugo Teisseire

---

## 📄 Licence

GPL v3 (héritée de SLAMTEC LIDAR SDK)
