# Système d'initialisation

## Vue d'ensemble

L'initialisation se fait via le **bouton BP_INIT** qui démarre automatiquement tous les programmes nécessaires.

## Processus d'initialisation

Quand vous appuyez sur **BP_INIT**, le système démarre dans cet ordre :

### 1. 📡 LIDAR
- Programme : `test_lidar_socket` (simulateur pour le moment)
- Argument : équipe (0=BLEU, 1=ROUGE) 
- Nom du process : `LIDAR_SIM`

### 2. 🔧 Programme sélectionnable (via switches SW_SEL)
- Sélection via 3 switches (SW_SEL_1, SW_SEL_2, SW_SEL_3)
- 8 combinaisons possibles (0-7)
- Configuration dans `config.py` → `SELECTABLE_PROGRAMS`

**Configuration actuelle :**
```python
SELECTABLE_PROGRAMS = {
    0: None,  # Aucun programme
    1: {'type': 'python', 'path': '...', 'name': 'PROG_1'},
    2: {'type': 'python', 'path': '...', 'name': 'PROG_2'},
    3: {'type': 'c', 'path': '...', 'name': 'PROG_3'},
    # etc.
}
```

### 3. 🎯 Programme STRATÉGIE
- Programme : `simu_strategy.py` (simulateur pour le moment)
- Nom du process : `STRATEGY`
- Gère la logique de match

## Résumé affiché

Après l'initialisation, le système affiche :
```
✅ INITIALISATION TERMINÉE
📊 3 processus actifs:
   🟢 LIDAR_SIM (PID: 1234)
   🟢 PROG_2 (PID: 1235)
   🟢 STRATEGY (PID: 1236)
```

## Modification de la configuration

### Ajouter un programme sélectionnable

Éditez `config.py` :

```python
SELECTABLE_PROGRAMS = {
    # ...
    4: {
        'type': 'python',  # ou 'c'
        'path': '/chemin/vers/votre_prog.py',
        'name': 'MON_PROG'
    },
}
```

### Changer le programme de stratégie

Éditez `config.py` :

```python
STRATEGY_EXECUTABLE = '/chemin/vers/votre_strategie.py'
```

## Callbacks associés

- **BP_INIT** → `on_bp_init_rising()` : Démarre tout
- **BP_RST_LIDAR** → `on_bp_rst_lidar_rising()` : Redémarre uniquement le LIDAR
- **BP_RST_STATE** → `on_bp_rst_state_rising()` : **Arrête TOUT et reset les états à ERROR**
- **SW_TEAM** → `on_sw_team_change()` : Change l'équipe (BLEU/ROUGE)

### Détail BP_RST_STATE

Quand vous appuyez sur **BP_RST_STATE** :
1. 🛑 Arrête **tous les processus** en cours (LIDAR, programmes sélectionnés, STRATÉGIE)
2. ⚠️  Réinitialise `state_lidar` à `STATE_ERROR`
3. ⚠️  Réinitialise `state_board` à `STATE_ERROR`
4. 💡 Met à jour les LEDs d'état (rouge)

**Utilisation typique :**
- Arrêt d'urgence avant un nouveau match
- Reset complet après un dysfonctionnement
- Retour à l'état initial avant nouvelle initialisation

## Tests

### Test du reset complet
```bash
cd /home/raspi/Desktop/CDF/carte_control
python3 test_reset.py
```

Sortie attendue :
```
📍 PHASE 1: Initialisation
[... démarrage processus ...]
📊 Processus actifs avant reset: 2
   🟢 LIDAR_SIM (PID: 1234)
   🟢 STRATEGY (PID: 1235)

📍 PHASE 2: Reset d'état
🛑 Arrêt de tous les processus en cours...
✅ Tous les processus arrêtés
📊 Processus actifs après reset: 0
✅ Tous les processus ont été arrêtés correctement
```

### Test manuel
1. Démarrer `main.py`
2. Régler les switches SW_SEL (par ex: 001 = programme 1)
3. Appuyer sur BP_INIT
4. Vérifier les logs

### Test en simulation
```bash
cd /home/raspi/Desktop/CDF/carte_control
python3 main.py
# Appuyer sur BP_INIT (simulé via mock)
```

## Architecture

```
BP_INIT pressé
    ↓
on_bp_init_rising()
    ↓
    ├─→ Démarre LIDAR (avec arg équipe)
    ├─→ Lit SW_SEL (0-7)
    ├─→ Démarre programme sélectionné
    └─→ Démarre STRATÉGIE
    ↓
Tous les processus tournent en parallèle
```
