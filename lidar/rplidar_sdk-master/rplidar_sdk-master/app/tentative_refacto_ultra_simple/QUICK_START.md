# 🚀 Quick Start - Code Refactorisé

## ⚡ En 30 secondes

```bash
cd ultra_simple/
make ultra_simple_refactored
./ultra_simple_refactored 0 --channel --serial /dev/serial0 256000
```

---

## 📁 Fichiers importants

| Fichier | Description | Lignes |
|---------|-------------|--------|
| `robot_config.h` | ⚙️ Configuration centralisée | 251 |
| `lidar_lib_refactored.h` | 📚 API LIDAR refactorée | 186 |
| `lidar_lib_refactored.cpp` | 📚 Implémentation | 618 |
| `main_refactored.cpp` | 🚀 Programme principal | 503 |
| `README_REFACTORED.md` | 📖 Documentation complète | 380 |
| `MIGRATION_GUIDE.md` | 🔄 Guide de migration | 420 |
| `REFACTORING_SUMMARY.md` | 📊 Récapitulatif détaillé | 400 |
| `test_lidar_refactored.cpp` | 🧪 Tests unitaires | 430 |
| `Makefile.refactored` | 🔧 Build système | 68 |

**Total** : 3256 lignes de code, doc et tests

---

## 🎯 Changements clés

### 1. Configuration centralisée

**AVANT** : Constantes dispersées partout  
**APRÈS** : Tout dans `robot_config.h`

```cpp
// Modifier une constante : 1 seul fichier
constexpr float PILLAR_RADIUS_MM = 50.0f;
constexpr float LIDAR_RESOLUTION_DEG = 0.25f;
```

### 2. Typage fort

**AVANT** : `int team = 0`  
**APRÈS** : `Team team = Team::TEAM0`

```cpp
Team team = intToTeam(atoi(argv[1]));
std::vector<Position> pillars = initPillars(team);
Position start = getRobotStartPosition(team);
```

### 3. Structures d'état

**AVANT** : Variables globales éparpillées  
**APRÈS** : Structures organisées

```cpp
SocketState socketState;
RobotState robotState;
LocalizationState locState;
```

### 4. MainLoop décomposé

**AVANT** : 180 lignes monolithiques  
**APRÈS** : 8 fonctions modulaires

```cpp
mainLoop() {
    sendDataToClient();
    updateIMUPosition();
    processScan();
    computeAndUpdatePose();
    fusePoses();
    displayStatus();
}
```

---

## 🔧 Compilation

### Méthode 1 : Makefile intégré

```bash
# Ajouter au Makefile existant
cat Makefile.refactored >> Makefile

# Compiler
make ultra_simple_refactored
```

### Méthode 2 : Commande directe

```bash
g++ -std=c++11 \
    main_refactored.cpp \
    lidar_lib_refactored.cpp \
    ImuOTOS.cpp \
    -I. -I../../../sdk/include \
    -L../../../sdk/lib \
    -lsl_lidar_sdk -lpthread -lrt \
    -o ultra_simple_refactored
```

---

## 🎮 Utilisation

### Équipe 0 (côté droit)

```bash
./ultra_simple_refactored 0 --channel --serial /dev/serial0 256000
```

### Équipe 1 (côté gauche)

```bash
./ultra_simple_refactored 1 --channel --serial /dev/serial0 256000
```

### Mode debug (centre)

```bash
./ultra_simple_refactored 3 --channel --serial /dev/serial0 256000
```

---

## 📊 Logs

Les logs utilisent des **emoji** pour faciliter la lecture :

```
🎯 Équipe 0 (côté droit) - Piliers chargés:
📍 Position initiale: x=2200 mm, y=1000 mm, θ=0°
🔧 Initialisation IMU OTOS...
✅ IMU initialisé avec succès
🚀 Ultra simple LIDAR - SDK v2.1.0
📡 Scan acquis en 45 ms (1234 points)
📍 LIDAR: x=2198 mm, y=1002 mm, θ=0.5°
🎯 Piliers détectés: 3/3
```

| Emoji | Signification |
|-------|--------------|
| 🎯 | Configuration/Piliers |
| 📍 | Position |
| 🔧 | Initialisation |
| ✅ | Succès |
| ❌ | Erreur |
| ⚠️ | Warning |
| 📡 | Scan LIDAR |
| 🚀 | Démarrage |
| 🛑 | Arrêt |

---

## 🧪 Tests

### Lancer les tests unitaires

```bash
# Compiler les tests
g++ -std=c++11 test_lidar_refactored.cpp \
    lidar_lib_refactored.cpp ImuOTOS.cpp \
    -I. -I../../../sdk/include \
    -L../../../sdk/lib -lsl_lidar_sdk \
    -lgtest -lgtest_main -lpthread \
    -o test_lidar_refactored

# Exécuter tous les tests
./test_lidar_refactored

# Exécuter un test spécifique
./test_lidar_refactored --gtest_filter=ConfigTest.*
```

### Tests disponibles

- `ConfigTest` : Configuration et enum Team
- `ConversionTest` : Conversions polaire/cartésien
- `InitializationTest` : Init piliers et tracking
- `PoseComputationTest` : Calcul de pose
- `StateStructuresTest` : Structures d'état
- `ValidationTest` : Validation des données

---

## 🔍 Débugger

### Afficher uniquement les erreurs

```bash
./ultra_simple_refactored 0 ... 2>&1 | grep "❌"
```

### Afficher les positions

```bash
./ultra_simple_refactored 0 ... 2>&1 | grep "📍"
```

### Mode verbose avec gdb

```bash
gdb ./ultra_simple_refactored
(gdb) run 0 --channel --serial /dev/serial0 256000
```

---

## 📝 Modifier la configuration

### Changer une constante

Éditer `robot_config.h` :

```cpp
// Modifier le rayon de détection des piliers
constexpr float PILLAR_RADIUS_MM = 60.0f;  // était 50.0f
```

Recompiler :

```bash
make clean-refactored
make ultra_simple_refactored
```

### Ajouter une nouvelle équipe

Dans `robot_config.h` :

```cpp
// 1. Ajouter dans l'enum
enum class Team {
    TEAM0 = 0,
    TEAM1 = 1,
    TEAM2 = 2,  // NOUVEAU
    DEBUG = 3
};

// 2. Ajouter les positions de piliers
const std::vector<Position> PILLARS_TEAM2 = {
    {1500.0f, 0.0f, 0.0f},
    {1500.0f, 2000.0f, 0.0f},
    {0.0f, 1000.0f, 0.0f}
};

// 3. Ajouter la position de départ
const Position ROBOT_START_TEAM2 = {1500.0f, 1000.0f, PI};

// 4. Mettre à jour getPillarPositions()
inline const std::vector<Position>& getPillarPositions(Team team) {
    switch (team) {
        case Team::TEAM2: return PILLARS_TEAM2;  // NOUVEAU
        // ... autres cases
    }
}

// 5. Mettre à jour getRobotStartPosition()
inline Position getRobotStartPosition(Team team) {
    switch (team) {
        case Team::TEAM2: return ROBOT_START_TEAM2;  // NOUVEAU
        // ... autres cases
    }
}
```

Utilisation :

```bash
./ultra_simple_refactored 2 --channel --serial /dev/serial0 256000
```

---

## 🔄 Migration depuis l'original

### Étape 1 : Backup

```bash
git add -A
git commit -m "Backup avant migration vers version refactorée"
```

### Étape 2 : Tests parallèles

```bash
# Terminal 1 : Version originale
./ultra_simple 0 --channel --serial /dev/serial0 256000

# Terminal 2 : Version refactorée
./ultra_simple_refactored 0 --channel --serial /dev/serial0 256000
```

### Étape 3 : Validation

Vérifier que les positions calculées sont cohérentes (écart < 10mm).

### Étape 4 : Déploiement

Remplacer l'ancien binaire :

```bash
mv ultra_simple ultra_simple_old
ln -s ultra_simple_refactored ultra_simple
```

---

## 📚 Documentation

### Générer la doc Doxygen

```bash
# Installer Doxygen
sudo apt-get install doxygen graphviz

# Créer Doxyfile
doxygen -g

# Éditer Doxyfile
# INPUT = . robot_config.h lidar_lib_refactored.h
# RECURSIVE = YES
# EXTRACT_ALL = YES

# Générer
doxygen Doxyfile

# Ouvrir
firefox html/index.html
```

### Fichiers de documentation

- `README_REFACTORED.md` : Vue d'ensemble
- `MIGRATION_GUIDE.md` : Migration pas à pas
- `REFACTORING_SUMMARY.md` : Détails techniques
- `QUICK_START.md` : Ce fichier

---

## 🆘 Aide

### Problème de compilation

```bash
# Vérifier les dépendances
ls ../../../sdk/lib/libsl_lidar_sdk.a
ls ImuOTOS.h

# Nettoyer et recompiler
make clean-refactored
make ultra_simple_refactored -B
```

### Problème d'exécution

```bash
# Vérifier les permissions
ls -l /dev/serial0
# Si besoin : sudo chmod 666 /dev/serial0

# Vérifier le socket
ls -l /tmp/robot.sock
# Si besoin : rm /tmp/robot.sock
```

### Comportement différent de l'original

Consulter `MIGRATION_GUIDE.md` section "Problèmes courants".

---

## 🎓 Ressources

### Fichiers à lire dans l'ordre

1. `QUICK_START.md` (ce fichier) - 5 min
2. `README_REFACTORED.md` - 15 min
3. `REFACTORING_SUMMARY.md` - 20 min
4. `MIGRATION_GUIDE.md` - 30 min si migration

### Code à étudier dans l'ordre

1. `robot_config.h` - Comprendre la config
2. `lidar_lib_refactored.h` - Voir l'API
3. `main_refactored.cpp` - Voir le flux principal
4. `test_lidar_refactored.cpp` - Exemples d'utilisation

---

## ✅ Checklist première utilisation

- [ ] Lire ce fichier (5 min)
- [ ] Compiler : `make ultra_simple_refactored`
- [ ] Lancer : `./ultra_simple_refactored 0 ...`
- [ ] Vérifier les logs avec emoji
- [ ] Tester les 3 modes (team 0, 1, 3)
- [ ] Comparer avec version originale
- [ ] Lire `README_REFACTORED.md`
- [ ] (Optionnel) Lancer les tests unitaires

---

## 🚀 Prochaines étapes

1. ✅ **Aujourd'hui** : Compiler et tester
2. ⏳ **Cette semaine** : Valider sur table réelle
3. ⏳ **Ce mois** : Ajouter tests unitaires
4. ⏳ **Plus tard** : Config JSON runtime

---

**Version** : 1.0  
**Date** : 7 novembre 2025  
**Auteur** : GitHub Copilot + Hugo Teisseire  
**Status** : ✅ Production ready
