# 🔄 Guide de Migration - Version Refactorée

## 📋 Checklist de migration

### Phase 1 : Préparation (5 min)
- [ ] Lire `README_REFACTORED.md` pour comprendre les changements
- [ ] Sauvegarder le code actuel : `git commit -am "Backup avant refactorisation"`
- [ ] Vérifier que le code original compile : `make ultra_simple`

### Phase 2 : Intégration Makefile (10 min)
- [ ] Ouvrir `Makefile` existant
- [ ] Copier le contenu de `Makefile.refactored` à la fin
- [ ] Adapter les variables `CXXFLAGS` et `LDFLAGS` si nécessaire
- [ ] Tester la compilation : `make ultra_simple_refactored`

### Phase 3 : Tests comparatifs (30 min)
- [ ] Lancer les deux versions en parallèle (voir section Tests)
- [ ] Comparer les sorties socket (logs Python)
- [ ] Vérifier la cohérence des positions calculées
- [ ] Mesurer les performances (temps de cycle)

### Phase 4 : Déploiement (15 min)
- [ ] Mettre à jour les scripts de lancement
- [ ] Adapter la configuration Python si nécessaire
- [ ] Déployer sur le robot
- [ ] Test final sur table

---

## 🔍 Correspondance des fichiers

| Fichier original | Fichier refactorisé | Statut |
|------------------|---------------------|--------|
| `lidar_lib.h` | `lidar_lib_refactored.h` | ✅ Compatible |
| `lidar_lib.cpp` | `lidar_lib_refactored.cpp` | ✅ Compatible |
| `main.cpp` | `main_refactored.cpp` | ✅ Compatible |
| N/A | `robot_config.h` | 🆕 Nouveau |
| `Makefile` | `Makefile.refactored` | 📝 À intégrer |

---

## ⚙️ Changements dans les API

### Types

```cpp
// AVANT
int team = 0;
position p;

// APRÈS
Team team = Team::TEAM0;
Position p;
```

### Constantes

```cpp
// AVANT (dans lidar_lib.cpp)
#define RAYON_PILIER 50
#define RESOLUTION 0.25f
#define NUM_ANGLES 1440

// APRÈS (dans robot_config.h)
constexpr float PILLAR_RADIUS_MM = 50.0f;
constexpr float LIDAR_RESOLUTION_DEG = 0.25f;
constexpr int LIDAR_NUM_ANGLES = 1440;
```

### Nommage des structures

```cpp
// AVANT
struct position {
    float x;
    float y;
    float angle;
};

// APRÈS
struct Position {
    float x_mm;      // Unité explicite
    float y_mm;
    float angle_rad;
};
```

### Fonctions

```cpp
// AVANT
void initPillars(int team);
position computePose(TrackResult* meas);

// APRÈS
std::vector<Position> initPillars(Team team);
Position computeRobotPose(const std::vector<TrackResult>& measurements,
                          const std::vector<Position>& pillars);
```

---

## 🧪 Tests de validation

### Test 1 : Compilation

```bash
# Terminal 1 : Version originale
make clean
make ultra_simple

# Terminal 2 : Version refactorée
make clean-refactored
make ultra_simple_refactored
```

**Critère de succès** : Les deux compilent sans warning

---

### Test 2 : Exécution basique

```bash
# Version originale
./ultra_simple 0 --channel --serial /dev/serial0 256000 &
ORIGINAL_PID=$!

# Version refactorée
./ultra_simple_refactored 0 --channel --serial /dev/serial0 256000 &
REFACTORED_PID=$!

# Attendre 30 secondes
sleep 30

# Arrêter les deux
kill $ORIGINAL_PID $REFACTORED_PID
```

**Critère de succès** : Les deux tournent sans crash

---

### Test 3 : Comparaison des sorties

Créer un script Python pour comparer les données socket :

```python
# test_compare_versions.py
import socket
import struct
import time

def read_socket(path):
    """Lit les données du socket"""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(path)
    
    data = sock.recv(72)  # 18 floats * 4 bytes
    if len(data) == 72:
        floats = struct.unpack('18f', data)
        return {
            'pos_imu': floats[0:3],
            'pos_lidar': floats[3:6],
            'pillar1': floats[6:9],
            'pillar2': floats[9:12],
            'pillar3': floats[12:15],
        }
    return None

# Comparer
original = read_socket('/tmp/robot.sock')
refactored = read_socket('/tmp/robot_refactored.sock')  # Adapter le chemin

if original and refactored:
    dx = abs(original['pos_lidar'][0] - refactored['pos_lidar'][0])
    dy = abs(original['pos_lidar'][1] - refactored['pos_lidar'][1])
    print(f"Δx={dx:.2f} mm, Δy={dy:.2f} mm")
    
    if dx < 10 and dy < 10:
        print("✅ Positions cohérentes")
    else:
        print("⚠️ Écart significatif")
```

**Critère de succès** : Écart < 10mm sur position

---

### Test 4 : Performance

Mesurer le temps de cycle avec `time` ou chronométrage intégré :

```bash
# Dans le code, ajouter des mesures
auto t_start = high_resolution_clock::now();
// ... boucle principale
auto t_end = high_resolution_clock::now();
auto duration = duration_cast<milliseconds>(t_end - t_start);
```

**Critère de succès** : Performance similaire (±5%)

---

## 🐛 Problèmes courants

### Erreur de compilation : "Team n'existe pas"

**Cause** : `robot_config.h` non inclus

**Solution** :
```cpp
#include "robot_config.h"  // Ajouter en haut du fichier
```

---

### Erreur de link : "undefined reference"

**Cause** : Objets manquants dans le Makefile

**Solution** :
```makefile
# Vérifier que tous les .o sont listés
ultra_simple_refactored: main_refactored.o lidar_lib_refactored.o ImuOTOS.o
```

---

### Valeurs différentes entre versions

**Cause** : Changement de structure `position` → `Position`

**Solution** : Vérifier les conversions, notamment :
```cpp
// Ancien code utilisant position.x (mm implicite)
// Nouveau code utilisant Position.x_mm (explicite)
```

---

### Socket ne se connecte pas

**Cause** : Chemin socket différent

**Solution** :
```cpp
// Vérifier dans robot_config.h
constexpr const char* SOCKET_PATH = "/tmp/robot.sock";
```

---

## 📊 Comparaison de performance

### Metrics à surveiller

| Métrique | Original | Refactorisé | Écart acceptable |
|----------|----------|-------------|------------------|
| Temps cycle (ms) | ~50 | ~50 | ±5 ms |
| Mémoire (MB) | ~15 | ~15 | ±2 MB |
| CPU (%) | ~30 | ~30 | ±5% |
| Détections/s | ~20 | ~20 | ±2 |

### Outils de mesure

```bash
# CPU et mémoire
top -p $(pidof ultra_simple_refactored)

# Temps d'exécution
time ./ultra_simple_refactored 0 --channel --serial /dev/serial0 256000

# Profiling (si disponible)
valgrind --tool=callgrind ./ultra_simple_refactored ...
```

---

## 🔐 Rollback

Si problème, revenir à la version originale :

```bash
# 1. Arrêter le processus refactorisé
sudo killall ultra_simple_refactored

# 2. Relancer l'original
./ultra_simple 0 --channel --serial /dev/serial0 256000

# 3. Git rollback si nécessaire
git checkout -- .
```

---

## 📝 Checklist post-migration

### Code
- [ ] Version refactorée compile sans warning
- [ ] Tests unitaires passent (si implémentés)
- [ ] Valgrind ne détecte pas de leak
- [ ] Code review effectué

### Fonctionnel
- [ ] Robot se localise correctement
- [ ] Détection des 3 piliers stable
- [ ] Fusion IMU/LIDAR fonctionne
- [ ] Communication socket OK avec Python

### Documentation
- [ ] README mis à jour
- [ ] Commentaires Doxygen vérifiés
- [ ] Guide utilisateur adapté
- [ ] Notes de version rédigées

### Déploiement
- [ ] Scripts de lancement mis à jour
- [ ] Configuration team0/team1/debug testée
- [ ] Test sur table réel effectué
- [ ] Backup version originale conservé

---

## 🎓 Formation équipe

### Points clés à expliquer

1. **Nouvelle structure** : `RobotState`, `LocalizationState`, `SocketState`
2. **Enum Team** : Utiliser `Team::TEAM0` au lieu de `0`
3. **robot_config.h** : Centralisation des constantes
4. **Nommage** : Unités explicites (`_mm`, `_rad`)
5. **Logs emoji** : Facilite le debug visuel

### Exercices pratiques

1. Modifier une constante dans `robot_config.h` et recompiler
2. Ajouter un nouveau pilier pour le mode DEBUG
3. Changer le seuil de vitesse pour la fusion IMU/LIDAR
4. Ajouter un log dans `displayStatus()`

---

## 📞 Support

En cas de problème :

1. **Vérifier les logs** : Les emoji facilitent l'identification
2. **Comparer avec l'original** : Lancer les deux versions en parallèle
3. **Documentation** : Consulter `README_REFACTORED.md`
4. **Debug** : Utiliser `gdb` ou ajouter des `std::cout`

---

## ✅ Validation finale

La migration est réussie si :

- ✅ Compilation sans warning
- ✅ Aucun crash pendant 10 minutes de fonctionnement
- ✅ Positions cohérentes avec version originale (< 10mm écart)
- ✅ Détection des 3 piliers stable
- ✅ Communication socket fonctionnelle
- ✅ Performance similaire

---

**Date de migration** : _________  
**Validé par** : _________  
**Rollback nécessaire** : ☐ Oui  ☑ Non
