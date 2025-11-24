# Architecture du Simulateur Robot Holonome

## 🎯 Vue d'ensemble

Ce programme simule un **robot holonome à 3 roues omnidirectionnelles** qui se déplace sur une table de 3m × 2m en évitant des obstacles fixes et un adversaire mobile. Il utilise l'algorithme **A*** pour calculer le meilleur chemin.

### Concept simple
```
┌─────────────────────────────────────────────────────┐
│  1. Robot veut aller à une cible                    │
│  2. A* calcule le meilleur chemin (évite obstacles) │
│  3. Robot suit le chemin point par point            │
│  4. Les roues tournent pour créer le mouvement      │
└─────────────────────────────────────────────────────┘
```

---

## 🏗️ Architecture en 4 couches

```
┌──────────────────────────────────────────────────────┐
│              COUCHE 4: AFFICHAGE                     │
│  Renderer: Dessine robot, obstacles, chemin          │
└──────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────┐
│        COUCHE 3: LOGIQUE DE SIMULATION               │
│  Simulator: Boucle principale, gestion événements    │
└──────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────┐
│         COUCHE 2: INTELLIGENCE (PATHFINDING)         │
│  avoidance.py: A* calcule chemin optimal             │
└──────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────┐
│         COUCHE 1: PHYSIQUE (CINÉMATIQUE)             │
│  holo_base.py: Calcul vitesses des 3 roues           │
└──────────────────────────────────────────────────────┘
```

---

## 📂 Structure des fichiers

### 1. `simulation_traj_clean.py` (Programme principal)
**Rôle**: Coordonne tout le programme

**Classes principales:**

#### A. `Config` (lignes 50-95)
Configuration centralisée de tous les paramètres.
```python
TABLE_WIDTH_MM = 3000        # Taille table
ROBOT_RADIUS_MM = 150        # Taille robot
MAX_WHEEL_SPEED_MPS = 0.3    # Vitesse max roues (30 cm/s)
TICKS_PER_SECOND = 60        # Fréquence simulation (60 fps)
```

#### B. `Utils` (lignes 100-135)
Fonctions de conversion coordonnées.
```python
to_screen(pos_mm)    # Convertit mm → pixels pour affichage
from_screen(pos_px)  # Convertit pixels → mm pour calculs
```

**Important**: Le système de coordonnées a (0,0) **en bas à gauche**, y+ vers le haut.

#### C. `SimulationState` (lignes 420-455)
Stocke tout l'état actuel de la simulation.
```python
self.robot              # Position, angle, vitesses robot
self.target_pos         # Où veut aller le robot
self.enemy_pos          # Position adversaire
self.path_astar         # Chemin calculé par A*
self.stuck_counter      # Compteur de blocage
```

#### D. `Renderer` (lignes 210-400)
Dessine tout à l'écran.
```python
_render_table()      # Dessine fond + obstacles
_render_robot()      # Dessine robot + roues
_render_path()       # Dessine chemin A* en cyan
_render_grid()       # Dessine grille occupation (AVEC CACHE)
```

**Optimisation clé**: La grille d'occupation est **mise en cache** ! 
- Cache recalculé uniquement quand la grille change
- Gain de performance: 85% (15ms → 2ms)

#### E. `Simulator` (lignes 460-700)
Contrôleur principal - **LE CŒUR DU PROGRAMME**.

**Méthode `run()` - Boucle principale:**
```python
while running:
    1. _handle_events()        # Clavier/souris
    2. _update_enemy()         # Bouge l'adversaire
    3. _update_pathfinding()   # Recalcule chemin A*
    4. _update_robot()         # Met à jour robot
    5. Renderer.render_all()   # Affiche tout
    6. clock.tick(60)          # 60 fps
```

---

### 2. `avoidance.py` (Pathfinding)
**Rôle**: Calcule le meilleur chemin pour éviter les obstacles

#### Concepts clés

##### A. Grille d'occupation
Le terrain est divisé en **cellules de 20mm × 20mm**.
```
Grille: 150 colonnes × 100 lignes = 15 000 cellules

┌─────────────────────────────┐
│ ☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐...   │  0 = libre
│ ☐☐☐■■■■☐☐☐☐☐☐☐☐☐☐☐☐☐...   │  1 = occupé
│ ☐☐☐■■■■☐☐☐☐☐☐☐☐☐☐☐☐☐...   │     (obstacle)
│ ☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐☐...   │
└─────────────────────────────┘
```

**Fonction `create_occupancy_grid()` (lignes 165-185)**
- Prend les obstacles rectangulaires
- Les agrandit du rayon du robot (150mm) → marge de sécurité
- Marque toutes les cellules touchées comme occupées (1)

##### B. Algorithme A*
**Fonction `astar()` (lignes 190-230)**

Trouve le **chemin le plus court** en explorant intelligemment.

```python
Principe:
1. Commence à la position robot (start_cell)
2. Explore les cellules voisines (8 directions)
3. Utilise heuristique pour prioriser (distance à la cible)
4. Évite cellules occupées (grid[x][y] == 1)
5. Retourne liste de cellules formant le chemin
```

**Heuristique diagonale (Chebyshev):**
```python
heuristic(a, b) = max(|ax - bx|, |ay - by|)
```
Plus rapide que distance euclidienne, admissible pour 8-directions.

**Mouvements possibles:**
```
  ↖  ↑  ↗
   \ | /
  ← · →     8 directions (4 cardinales + 4 diagonales)
   / | \
  ↙  ↓  ↘

Coût: 1.0 (cardinal), √2 ≈ 1.414 (diagonal)
```

##### C. Fonction principale: `compute_direction_astar()` (lignes 240-290)

**Pipeline complet:**
```python
1. Validation cible
   ├─ Dans obstacle ? → Code 0 (erreur)
   ├─ Sur ennemi ?     → Code 1 (adapter)
   └─ Libre ?          → Code 2 (OK)

2. Création grille occupation
   └─ Obstacles fixes + ennemi + marges sécurité

3. Gestion deadzone (robot bloqué)
   ├─ Robot dans cellule occupée ?
   ├─ → Chercher cellule libre proche
   └─ → Sinon: échappement d'urgence

4. Calcul A*
   └─ Retourne liste cellules: [(x1,y1), (x2,y2), ...]

5. Direction vers prochaine cellule
   └─ Normalise vecteur: robot.pos → prochaine_cellule
```

**Codes de retour:**
- `0`: Cible dans obstacle (STOP)
- `1`: Cible sur ennemi (contourner)
- `2`: Chemin libre (GO)
- `4`: Cible inaccessible (STOP)

##### D. Échappement deadzone
**Fonction `get_escape_direction()` (lignes 310-340)**

Appelée quand robot coincé dans zone interdite.

**Stratégie:**
```python
1. Calculer répulsion depuis l'ennemi (× 2.0)
2. Calculer répulsion depuis obstacles proches (< 300mm)
3. Sommer toutes les répulsions
4. Normaliser → direction d'échappement
5. Si aucune direction claire → aléatoire
```

---

### 3. `holo_base.py` (Cinématique)
**Rôle**: Mathématiques du robot à 3 roues holonomes

#### A. Configuration robot
```
        Roue 1 (90°)
            ↑
           / \
          /   \
         /  ·  \    · = centre robot
        /       \   
       /         \
    Roue 2      Roue 3
    (210°)      (330°)
```

**Angles des roues:**
- Roue 1: 90° (arrière, verticale)
- Roue 2: 210° (avant-gauche, -30° de 180°)
- Roue 3: 330° (avant-droite, +30° de 0°)

Espacement: 120° entre chaque roue (symétrie parfaite).

#### B. Cinématique directe: `compute_wheel_speeds_global()`

**Problème résolu:**
> "Je veux que le robot aille dans la direction (vx, vy).
> À quelle vitesse chaque roue doit tourner ?"

**Algorithme:**
```python
1. Rotation coordonnées globales → locales robot
   v_local = rotation(-robot.angle) · [vx, vy]
   
   Pourquoi ? Les roues "voient" le monde selon orientation robot

2. Projection sur chaque roue
   Pour chaque roue à angle θ:
   vitesse_roue = -sin(θ)·vx_local + cos(θ)·vy_local + ω·R
   
   Explication:
   - Composante tangentielle roue
   - + contribution rotation (omega)
   
3. Normalisation (max = 1.0)
   Si max(vitesses) > 1.0 → diviser tout par max
   
   Garantit: aucune roue ne dépasse 100% capacité
```

**Exemple concret:**
```python
# Robot orienté à 45° veut aller tout droit vers le haut (0, 1)
vx_global, vy_global = 0.0, 1.0
robot.angle = 45° = π/4 rad

# Étape 1: Rotation
cos(45°) = sin(45°) = 0.707
vx_local = 0.707 * 0 - 0.707 * 1 = -0.707
vy_local = 0.707 * 0 + 0.707 * 1 =  0.707

# Étape 2: Projection
Roue 1 (90°):  -sin(90°)·(-0.707) + cos(90°)·0.707 = 0
Roue 2 (210°): -sin(210°)·(-0.707) + cos(210°)·0.707 = -0.966
Roue 3 (330°): -sin(330°)·(-0.707) + cos(330°)·0.707 =  0.966

→ Roues 2 et 3 tournent en sens opposé → translation
```

#### C. Cinématique inverse: `compute_base_velocity()`

**Problème inverse:**
> "Les roues tournent à [s1, s2, s3].
> Quelle est la vitesse du robot (vx, vy) ?"

**Algorithme (odométrie):**
```python
Pour chaque roue i à angle θi:
   vx_local += -sin(θi) · vitesse_roue_i
   vy_local +=  cos(θi) · vitesse_roue_i

Retourne [vx_local, vy_local]  # Dans repère robot
```

**Utilisation:** Calculer déplacement réel du robot (simulation physique).

#### D. Rotation: `compute_rotation_velocity()`

**Formule:**
```python
ω = (v1 + v2 + v3) / (3 · R)
```

**Logique:**
- Si toutes roues tournent dans même sens → rotation sur place
- Somme positive → rotation antihoraire
- Somme négative → rotation horaire
- Somme nulle → translation pure

---

## 🔄 Flux d'exécution complet

### Boucle principale (60 fois par seconde)

```
┌─────────────────────────────────────────────┐
│ 1. ENTRÉES UTILISATEUR                      │
│    - Clic souris: déplacer cible/ennemi     │
│    - T: toggle cible/ennemi                 │
│    - S: start/stop robot                    │
│    - H: téléporter robot                    │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 2. MISE À JOUR ENNEMI                       │
│    - Toutes les 2s: nouvelle direction      │
│    - Déplacement 5mm/frame                  │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 3. PATHFINDING (toutes les 15 frames)      │
│                                             │
│    compute_direction_astar()                │
│    ├─ Créer grille occupation               │
│    ├─ Vérifier deadzone                     │
│    ├─ Calculer A*                           │
│    └─ Extraire direction                    │
│                                             │
│    Sortie: path_astar = [(x1,y1), ...]     │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 4. SUIVI DE CHEMIN                          │
│                                             │
│    update_path_following()                  │
│    ├─ Prendre prochaine cellule du chemin   │
│    ├─ Si distance < 150mm → cellule suivante│
│    └─ Calculer direction normalisée         │
│                                             │
│    Sortie: direction = [dx, dy]             │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 5. SÉCURITÉ                                 │
│                                             │
│    - Distance ennemi < 350mm ? → STOP       │
│    - Bloqué > 0.5s ? → Téléport 100mm      │
│    - Cible dans obstacle ? → PAUSE          │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 6. CINÉMATIQUE                              │
│                                             │
│    compute_wheel_speeds_global()            │
│    ├─ direction [dx, dy] + omega            │
│    ├─ Rotation global → local              │
│    ├─ Projection sur 3 roues                │
│    └─ Normalisation                         │
│                                             │
│    Sortie: [speed1, speed2, speed3]         │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 7. LISSAGE VITESSES                         │
│                                             │
│    Moyenne mobile sur 5 frames:             │
│    wheel_speed_final[i] = mean(             │
│       historique[-5:])                      │
│                                             │
│    Évite à-coups mécaniques                 │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 8. MISE À JOUR POSITION                     │
│                                             │
│    compute_base_velocity()                  │
│    ├─ wheel_speeds → velocity_local         │
│    └─ Rotation → velocity_global            │
│                                             │
│    robot.pos += velocity_global             │
│    robot.angle += rotation_velocity         │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 9. VÉRIFIER OBJECTIF ATTEINT                │
│                                             │
│    Si distance(robot, cible) < 170mm:       │
│       - Nouvelle cible aléatoire            │
│       - Recalculer A*                       │
└─────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────┐
│ 10. AFFICHAGE                               │
│                                             │
│     Renderer.render_all()                   │
│     ├─ Table + obstacles                    │
│     ├─ Grille occupation (CACHE)            │
│     ├─ Chemin A* (lignes cyan)              │
│     ├─ Robot + roues                        │
│     ├─ Ennemi (cercle rouge)                │
│     ├─ Cible (cercle bleu)                  │
│     └─ UI (vitesses, erreurs)               │
│                                             │
│     pygame.display.flip()                   │
└─────────────────────────────────────────────┘
              ↓
         clock.tick(60)  ← Attendre pour maintenir 60 fps
              ↓
         (Recommencer)
```

---

## 🔍 Mécanismes importants expliqués

### 1. Détection de blocage (Deadzone)

**Problème:** Robot peut se retrouver "coincé" dans une cellule marquée occupée.

**Causes possibles:**
- Erreur de positionnement (simulation)
- Recalcul grille pendant mouvement
- Ennemi se déplace sur robot

**Solution 1: Compteur de blocage** (`_update_robot()`, lignes 545-565)
```python
# Chaque frame:
movement = distance(position_actuelle, position_précédente)

if movement < 1mm and robot_moving:
    stuck_counter++
else:
    stuck_counter = 0

# Si bloqué 30 frames (0.5s @ 60fps):
if stuck_counter > 30:
    - Téléporter robot 100mm direction aléatoire
    - Réinitialiser compteur
    - Forcer recalcul A*
```

**Solution 2: Échappement intelligent** (`get_escape_direction()`)
```python
Si robot dans cellule occupée:
    1. Trouver cellule libre la plus proche
    2. Si toujours occupé:
       - Calculer direction fuite (oppose ennemi + obstacles)
       - Retourner cette direction au lieu du A*
```

### 2. Cache de la grille d'occupation

**Problème:** Dessiner 15 000 rectangles à chaque frame = 15ms (trop lent).

**Solution:** Cache basé sur hash (`_render_grid()`, lignes 330-370)

```python
# Calculer "empreinte" de la grille
grid_hash = hash(tuple(tuple(row) for row in grid))

if grid_hash != cached_grid_hash:
    # Grille a changé → régénérer surface
    1. Créer surface pygame transparente
    2. Dessiner tous les rectangles sur cette surface
    3. Sauvegarder surface + hash
    
else:
    # Grille inchangée → utiliser cache
    1. Blit rapide de la surface sauvegardée
    
→ Gain: 15ms → 2ms (85% plus rapide)
```

**Quand la grille change-t-elle ?**
- Ennemi bouge → son rectangle obstacle change
- Recalcul A* (toutes les 15 frames)

### 3. Lissage des vitesses

**Problème:** Changements brusques de direction → vitesses roues oscillent.

**Solution:** Moyenne mobile sur 5 frames (`_update_robot()`, lignes 600-615)

```python
Pour chaque roue i:
    historique[i].append(nouvelle_vitesse)
    
    if len(historique[i]) > 5:
        historique[i].pop(0)  # Garder seulement 5 dernières
    
    vitesse_finale = moyenne(historique[i])
```

**Effet:**
```
Sans lissage:    ⎯⎯╱╲⎯╲╱⎯⎯╱╲⎯   (oscillations)
Avec lissage:    ⎯⎯⎯╱‾‾‾╲⎯⎯⎯   (lisse)
```

**Fenêtre de 5 frames = 83ms** → compromis entre réactivité et stabilité.

### 4. Sécurité distance ennemi

**Règle:** Ne jamais s'approcher à moins de 350mm de l'adversaire.

**Calcul:** (`_update_robot()`, lignes 575-585)
```python
safety_distance = robot_radius + enemy_radius + marge
                = 150mm + 150mm + 50mm = 350mm

dist_to_enemy = ||robot.pos - enemy.pos||

if dist_to_enemy < safety_distance:
    - Arrêter toutes les roues (vitesse = 0)
    - Lever pause_robot = True
    - Afficher "Trop proche de l'adversaire"
```

**Pourquoi important ?**
- Évite collision physique
- Laisse marge erreur capteurs (sur robot réel)
- Conforme règlement compétition robotique

### 5. Recalcul périodique A*

**Pourquoi ne pas calculer qu'une fois ?**
- Ennemi bouge → obstacles changent
- Robot peut dévier légèrement → recalage
- Opportunités: chemin plus court peut apparaître

**Stratégie:** (`_update_pathfinding()`, lignes 490-520)
```python
if not path_computed:
    # Calculer A* (coûteux)
    path, code = compute_direction_astar(...)
    path_computed = True
    recompute_counter = 0
else:
    recompute_counter++
    
    if recompute_counter > 15:  # 15 frames = 250ms
        path_computed = False  # Forcer recalcul au prochain tour
```

**Compromis:**
- Trop fréquent: gaspille CPU
- Trop rare: chemin obsolète
- **15 frames (250ms)** = bon équilibre

---

## 🎮 Contrôles clavier

| Touche | Action | Effet |
|--------|--------|-------|
| **Clic gauche** | Déplacer | Cible (si T activé) ou Ennemi |
| **T** | Toggle mode | Basculer entre déplacement cible/ennemi |
| **S** | Start/Stop | Démarrer ou arrêter le robot |
| **H** | Home | Téléporter robot à position souris |
| **G** | Grid | Afficher/cacher lignes grille A* |
| **O** | Occupancy | Afficher/cacher cellules occupées |
| **ESC** | Quitter | Fermer la simulation |

---

## 📊 Métriques de performance

### Temps de calcul typiques (60 fps = 16.67ms/frame max)

| Opération | Temps | Fréquence | Impact |
|-----------|-------|-----------|--------|
| **A* complet** | ~3-5ms | 1× toutes les 15 frames | Moyen |
| **Grille occupation** | ~0.5ms | Avec A* | Faible |
| **Cinématique** | ~0.1ms | Chaque frame | Négligeable |
| **Render (sans cache)** | ~15ms | Chaque frame | ❌ CRITIQUE |
| **Render (avec cache)** | ~2ms | Chaque frame | ✅ OK |
| **Total boucle** | ~3-4ms | Chaque frame | ✅ 60fps stable |

### Optimisations clés appliquées

1. ✅ **Cache surface occupancy**: 85% gain render
2. ✅ **Grille haute résolution (20mm)**: Précision vs performance
3. ✅ **Recalcul A* espacé (15 frames)**: Économise CPU
4. ✅ **Désactivation lignes grille** (`SHOW_GRID_LINES=False`): Gain 2-3ms
5. ✅ **Heuristique Chebyshev**: Plus rapide que Euclidienne

---

## 🧩 Diagramme de classes

```
┌─────────────────┐
│     Config      │  Configuration globale
└─────────────────┘

┌─────────────────┐
│      Utils      │  Fonctions utilitaires
├─────────────────┤
│ to_screen()     │
│ from_screen()   │
│ random_position()│
└─────────────────┘

┌─────────────────┐
│  SimulationState│  État de la simulation
├─────────────────┤
│ robot           │
│ target_pos      │
│ enemy_pos       │
│ path_astar      │
│ stuck_counter   │
└─────────────────┘

┌─────────────────┐      ┌─────────────────┐
│    Renderer     │◄─────┤   Simulator     │  Contrôleur
├─────────────────┤      ├─────────────────┤
│ render_all()    │      │ run()           │
│ _render_table() │      │ _update_robot() │
│ _render_grid()  │      │ _handle_events()│
│ (+ cache)       │      └─────────────────┘
└─────────────────┘             │ utilise
                                │
                    ┌───────────┴────────────┐
                    ↓                        ↓
            ┌─────────────────┐      ┌─────────────────┐
            │  avoidance.py   │      │  holo_base.py   │
            ├─────────────────┤      ├─────────────────┤
            │ astar()         │      │ compute_wheel_  │
            │ create_grid()   │      │   speeds()      │
            │ get_escape_dir()│      │ compute_base_   │
            └─────────────────┘      │   velocity()    │
                                     └─────────────────┘
```

---

## 🚀 Comment démarrer

### 1. Installer dépendances
```bash
pip install pygame numpy
```

### 2. Lancer simulation
```bash
python simulation_traj_clean.py
```

### 3. Tester fonctionnalités
```
1. Appuyer S → Robot démarre
2. Clic gauche → Déplacer cible
3. Robot calcule chemin A* (lignes cyan)
4. Robot suit le chemin en évitant obstacles
5. Appuyer T puis clic → Déplacer ennemi
6. Robot recalcule pour éviter ennemi
```

---

## 🔧 Paramètres ajustables

### Vitesse robot
```python
# simulation_traj_clean.py, ligne ~70
MAX_WHEEL_SPEED_MPS = 0.3  # Augmenter → robot plus rapide
```

### Fréquence recalcul A*
```python
# simulation_traj_clean.py, ligne ~80
PATH_RECOMPUTE_INTERVAL = 15  # Diminuer → recalcul plus fréquent
```

### Sensibilité blocage
```python
# simulation_traj_clean.py, ligne ~550
if stuck_counter > 30:  # Diminuer → déblocage plus rapide
```

### Résolution grille
```python
# avoidance.py, ligne ~15
CELL_SIZE = 20  # Diminuer → plus précis mais plus lent
```

### Distance sécurité ennemi
```python
# simulation_traj_clean.py, ligne ~577
safety_distance = ... + 50  # Augmenter → plus prudent
```

---

## 📚 Ressources complémentaires

### Algorithmes
- [Pathfinding A*](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Robot holonome](https://en.wikipedia.org/wiki/Holonomic_(robotics))
- [Cinématique inverse](https://en.wikipedia.org/wiki/Inverse_kinematics)

### Fichiers du projet
- `README_ARCHITECTURE.md`: Guide portage robot réel
- `README.md`: Documentation générale
- `SCHEMA_SYSTEME.md`: Architecture système globale

---

## ❓ FAQ

### Q: Pourquoi A* au lieu de champ de potentiel ?
**R:** A* garantit chemin optimal et gère mieux les deadlocks. Champ de potentiel peut créer minima locaux.

### Q: Pourquoi 3 roues holonomes ?
**R:** 3 roues = minimum pour holonomie (translation + rotation simultanées). Plus stable que 4 roues.

### Q: Pourquoi cache grille si elle change souvent ?
**R:** Même si recalcul A* toutes les 15 frames, grille reste identique ~93% du temps (14 frames sur 15).

### Q: Le lissage crée du retard ?
**R:** Oui, ~83ms (5 frames). Compromis acceptable pour éviter oscillations mécaniques.

### Q: Pourquoi 60 fps et pas 30 ?
**R:** Plus fluide visuellement + meilleur contrôle (lissage, détection blocage).

---

## 🎓 Pour aller plus loin

### Améliorations possibles

1. **Prédiction mouvement ennemi**
   - Estimer trajectoire future
   - A* évite position prédite (pas actuelle)

2. **Replanning dynamique**
   - D* Lite: recalcul incrémental (pas tout A*)
   - Plus efficace si petit changement

3. **Lissage trajectoire**
   - Splines cubiques entre points A*
   - Trajectoire plus naturelle

4. **Multi-objectifs**
   - Planificateur haut niveau (séquence cibles)
   - Optimisation ordre de visite (TSP)

5. **Apprentissage**
   - RL pour stratégies adversariales
   - Apprendre patterns évitement efficaces

---

**Auteur:** Simulateur Robot CDF  
**Date:** Novembre 2025  
**Version:** 2.0 (avec optimisations cache + lissage)
