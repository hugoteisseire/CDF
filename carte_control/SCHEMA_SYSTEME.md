# Schéma complet du système de contrôle

## 🎮 Boutons et leurs actions

```
┌──────────────────────────────────────────────────────────────┐
│                    PANNEAU DE CONTRÔLE                        │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  🔘 BP_INIT           → Démarre TOUT                         │
│                         • LIDAR                               │
│                         • Programme sélectionné (sw_sel 0-7)  │
│                         • STRATÉGIE                           │
│                                                               │
│  🔘 BP_RST_LIDAR      → Redémarre LIDAR uniquement          │
│                                                               │
│  🔘 BP_RST_STATE      → RESET COMPLET                       │
│                         • Arrête TOUS les processus           │
│                         • États → ERROR (LED rouge)           │
│                                                               │
│  🔘 TIRETTE           → Démarrage match (à implémenter)      │
│                                                               │
│  🎛️  SW_TEAM          → Sélection équipe BLEU/ROUGE         │
│                                                               │
│  🎛️  SW_SEL (3 bits)  → Sélection programme 0-7             │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

## 🔄 Cycle de vie typique

```
┌─────────────────────────────────────────────────────────────┐
│ 1. DÉMARRAGE SYSTÈME                                        │
│    python3 main.py                                          │
│    • Threads démarrés (actuator, poller, lidar_socket)     │
│    • États: ERROR                                           │
│    • Processus externes: aucun                              │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. CONFIGURATION                                            │
│    • Régler SW_TEAM (0=BLEU, 1=ROUGE)                      │
│    • Régler SW_SEL (000 à 111 = prog 0 à 7)                │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. INITIALISATION (BP_INIT)                                │
│    ✅ LIDAR démarré                                         │
│    ✅ Programme sélectionné démarré (si configuré)          │
│    ✅ STRATÉGIE démarrée                                    │
│    • États: OK/WARNING selon état LIDAR                    │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. MATCH EN COURS                                           │
│    • Tous les processus actifs                              │
│    • Monitoring I2C (10Hz)                                  │
│    • Réception états LIDAR via socket                       │
│    • LED d'état mises à jour en temps réel                  │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ 5. FIN / PROBLÈME                                           │
│    Option A: Ctrl+C → Arrêt propre                         │
│    Option B: BP_RST_STATE → Reset complet                  │
│    • Tous processus arrêtés                                 │
│    • États → ERROR                                          │
│    • LED rouge                                              │
└─────────────────────────────────────────────────────────────┘
                          ↓
              Retour à étape 2 (reconfiguration)
```

## 📊 États des LEDs

### LED LIDAR (RGB)
```
🔴 Rouge   → STATE_ERROR   (LIDAR down, erreur communication)
🟠 Orange  → STATE_WARNING (LIDAR init, lost)
🟢 Vert    → STATE_OK      (LIDAR running)
```

### LED État général (RGB)
```
🔴 Rouge   → STATE_ERROR   (Système en erreur, I2C down)
🟠 Orange  → STATE_WARNING (Avertissement)
🟢 Vert    → STATE_OK      (Tout fonctionne)
```

### LED Équipe (RGB)
```
🔵 Bleu    → Équipe BLEU
🔴 Rouge   → Équipe ROUGE
```

## 🔌 Communication inter-processus

```
┌──────────────────────────────────────────────────────────┐
│                    PROGRAMME PYTHON                       │
│                      (main.py)                           │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  Thread Principal         Thread Actuator                │
│  • Boucle principale      • LED updates                  │
│  • Affichage              • I2C monitoring               │
│                                                           │
│  Thread Poller            Thread LidarSocket             │
│  • GPIO polling           • Serveur socket UNIX          │
│  • Callbacks              • /tmp/robot.sock              │
│                           • Reçoit états LIDAR           │
└──────────────────────────────────────────────────────────┘
                              ↑↓
                    Socket UNIX: /tmp/robot.sock
                              ↑↓
┌──────────────────────────────────────────────────────────┐
│              PROGRAMMES EXTERNES (Processus)             │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  LIDAR_SIM (C)         PROG_1..7 (Python/C)             │
│  • Tracking            • Fonctionnalités                 │
│  • Envoie états        • Spécifiques                     │
│                                                           │
│  STRATEGY (Python)                                       │
│  • Logique de match                                      │
│  • Décisions                                             │
│                                                           │
└──────────────────────────────────────────────────────────┘
```

## 📁 Fichiers clés

| Fichier | Rôle |
|---------|------|
| `main.py` | Point d'entrée, orchestration |
| `callbacks.py` | Actions des boutons |
| `control_board.py` | Gestion MCP23017, LED, GPIO |
| `board_actuation.py` | Thread LED + monitoring I2C |
| `input_poller.py` | Thread polling GPIO + debounce |
| `process_manager.py` | Gestion processus externes |
| `unix_socket_thread.py` | Serveur socket + LidarSocket |
| `config.py` | Configuration globale |

## 🧪 Tests disponibles

```bash
# Test initialisation
python3 test_init.py

# Test reset complet
python3 test_reset.py

# Test modules
python3 test_modules.py
```
