# Localisation lidar (RPLIDAR C1 + odométrie + EKF)

## Organisation

| Dossier / fichier | Rôle |
|---|---|
| `core/` | Code de production : acquisition (`lidar_acquisition.py`, `lidar_process.py` = lecture série dans un processus séparé), clusters, association aux poteaux, EKF (`ekf.py`), localisation (`localizer.py`), relocalisation par triangulation (`relocalize.py`), moniteur CPU par thread (`thread_monitor.py`). |
| `rplidarc1/` | Bibliothèque du lidar (protocole série, lecture par blocs). |
| `tests/robot_fusion.py` | **Programme du robot** : moteurs CAN + lidar + EKF + serveur stratégie + page web (`--visualize` = fenêtre pygame de debug). |
| `tests/test_ekf_steps_0_3.py`, `tests/test_ekf_delayed_correction.py`, `tests/test_rplidar_protocol.py` | Tests automatiques (pytest). |
| `test_bas_niveau/` | Outils et tests bas niveau, sans EKF : `lidar_polar.py` (affichage polaire direct), `lidar_core.py` + `lidar_demo.py` (acquisition simple + clusters), `lidar_sim.py` (faux lidar), `test_lidar_core.py` (tests sans matériel). Se lancent depuis ce dossier : `python3 lidar_demo.py --sim`. |
| `../legacy/lidar/` | Anciens programmes, plus utilisés : `test_fusion.py`, `test_fusion_bck.py`, `robot_fusion_vector.py`, `bck/robot_fusion.py`, `benchmark_ekf_history.py`. |

## Lancer le robot

Depuis la racine du dépôt, **en SSH** (voir « Freezes » ci-dessous) :

```bash
python -m lidar.tests.robot_fusion --x0 2870 --y0 1000 --theta0-deg 90 --odometry-hz 50
```

Options utiles : `--visualize`, `--lidar-zero-deg`, `--win-scale`, `--opponent-inset-mm` (balise adverse cherchée seulement dans la table réduite de cette marge, 200 mm par défaut).

## Lire les logs

- `Tour N | ... calcul X (cpu Y attente Z) ... file F e2e E | dt D perdus a/b` : `calcul` doit rester bien sous 100 ms (période du lidar), `file` sous ~50 ms. `attente` >> `cpu` = le thread attend le processeur ou le GIL.
- `[CPU]` (toutes les 2 s) : %CPU de chaque thread et du processus lidar, et `charge` de la Pi (doit rester sous ~4 sur 4 cœurs).
- `[VIZ robot]` / `[SERVEUR]` / cadre « diag » de la page web : chaîne d'affichage robot -> serveur -> navigateur.
- `[LOCALIZER] relocalisé par triangulation ...` : le filtre était perdu et a repris sa pose à partir des poteaux.

## Freezes : causes déjà trouvées (novembre 2026)

1. **EKF** : l'historique coûtait O(N²) par échantillon d'odométrie (N ≈ fréquence odo), donc O(f³) par seconde. Corrigé : `EkfParams.history_min_dt_s` (20 ms) borne l'historique à ~50 états.
2. **Lecture série** : la bibliothèque lisait le port trame par trame et ne rendait pas la main à asyncio ; les tours arrivaient par rafales, déjà trop vieux. Corrigé : lecture par blocs (`rplidarc1/protocol.py`).
3. **Sous-tension** : quand les moteurs tournent, la Pi bridait son processeur (calcul ~3x plus lent). Corrigé par l'alimentation officielle. Vérifier avec `vcgencmd get_throttled` (doit valoir `0x0`).
4. **Bureau à distance** : `rpi-connect` + `wayvnc` + le terminal graphique qui fait défiler les logs consommaient ~1,4 cœur (charge ~7). Corrigé : piloter en SSH, partage d'écran coupé pendant les tests.

Si des freezes reviennent : regarder d'abord `charge` et `[CPU]`, puis `ps -eo pid,ppid,pcpu,etime,cmd --sort=-pcpu | head -15` et `vcgencmd get_throttled`, avant de toucher au code.

## Localisation : comportements à connaître

- **Filtre hors zone** : les impacts hors de la table + 200 mm sont ignorés ; la marge monte jusqu'à 400 mm quand un poteau n'est plus recalé depuis 1 s (`field_filter_margin_lost_mm`, `field_filter_widen_after_s`, `field_filter_widen_ramp_s`).
- **Fenêtres de recherche** : centrées sur la position théorique ; quand un seul poteau est vu, la covariance s'allonge et les fenêtres des autres poteaux peuvent devenir très larges (jusqu'à ±1500 mm / ±62° avec les réglages par défaut de `robot_fusion`). La fenêtre pygame affiche leur taille.
- **Perdu -> relocalisation** : si des poteaux sont vus mais toutes les corrections refusées pendant `lost_after_s` (typiquement un cap faux de 15-20° alors que le filtre se croit précis), la pose est recalculée par triangulation sur tous les points du tour (`RelocParams.enabled`).
- **Deux poteaux** suffisent à l'EKF (simulé : ~6-8 mm). Un mauvais recalage à deux poteaux vient plutôt d'un poteau effacé par le filtre hors zone, d'une association erronée dans une fenêtre trop large, ou d'un biais (rayon de poteau, `--lidar-zero-deg`).
