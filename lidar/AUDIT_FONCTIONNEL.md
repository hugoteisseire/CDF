# Audit fonctionnel — chaîne de localisation lidar

## Périmètre

Lecture statique de la chaîne `LidarAcquisition` → `Revolution` → filtrage géométrique → clusters → association aux poteaux → EKF. Cet audit ne remplace pas un essai avec le lidar réel ni les tests de simulation.

## Chemin d'une mesure

1. **Réception série — `core/lidar_acquisition.py`**
   - Le thread asyncio de la bibliothèque lit les trames et les dépose dans une deque.
   - Le thread de segmentation récupère l'angle, la distance, la qualité et `t_rx_ns`, puis convertit cet horodatage en epoch via `ClockAligner`.
   - Un tour est ouvert au passage 360° → 0°. Les tours trop courts, incomplets ou mal alignés sont rejetés.
   - Si `rev_queue` est pleine, le tour le plus ancien est retiré et remplacé par le plus récent : la latence reste bornée, mais des tours peuvent être perdus (compteur `revs_dropped`).

2. **Prétraitement — `core/localizer.py` et `core/lidar_clusters.py`**
   - `cap_points` retire les distances hors limites.
   - `_filter_points_in_field` projette chaque impact dans le repère terrain à l'aide de l'état EKF interpolé à l'horodatage du point, puis retire les impacts hors terrain élargi.
   - Les points dont aucun historique n'existe sont retirés. Les états `clamped` sont conservés mais comptés dans `points_unverified` : une désynchronisation peut donc laisser passer une projection approximative.
   - `find_clusters` regroupe les points voisins. Le timestamp du cluster est la moyenne des timestamps de ses points.

3. **Association — `core/pole_association.py`**
   - Les clusters sont filtrés par nombre de points et largeur attendue d'un poteau.
   - Pour chaque cluster, la pose à son timestamp est utilisée pour prédire distance et angle; des fenêtres adaptatives définissent les candidats.
   - Un cluster revendiqué par plusieurs poteaux, ou plusieurs clusters candidats pour le même poteau, est marqué `ambiguous`; il ne déclenche pas de correction.
   - Un état historique manquant est maintenant traité explicitement au lieu de déréférencer `None`.

4. **Correction — `core/localizer.py` puis `core/ekf.py`**
   - Seuls les matches `matched` passent à `correct_pole`; la distance est convertie de la surface au centre du poteau avec `pole_radius_mm`.
   - L'innovation contient distance et angle. La correction est rejetée si le chi² dépasse le seuil ou si le déplacement calculé dépasse les limites de sécurité.
   - Le timestamp du cluster est transmis à l'EKF; `lidar_latency_s` est soustrait dans `correct_pole`.
   - **Correction apportée :** le gain d'une mesure retardée est maintenant calculé avec les covariances croisées entre l'état observé et chaque état historique. La correction est propagée jusqu'à la pose courante; les covariances de l'historique et leurs covariances croisées sont mises à jour ensemble. Une mesure clairement hors de l'historique est refusée plutôt que corrigée à partir d'un état éloigné.

## Défauts corrigés dans cette branche

- **Gain incohérent pour une mesure retardée :** auparavant, `S` était calculé à partir de `P_ref`, mais le gain utilisait la covariance courante `self.P`. Ces deux matrices ne décrivent pas le même instant et le gain n'était pas le bon pour une mesure retardée.
- **Historique partiellement corrigé :** auparavant, un décalage identique était appliqué à toutes les poses et seule la dernière covariance était remplacée. Les poses et incertitudes historiques pouvaient donc devenir incohérentes.
- **Absence d'historique dans l'association :** `state_at` pouvait renvoyer `None`, utilisé ensuite sans vérification.
- **Tests de régression ajoutés :** correction retardée pendant un mouvement, symétrie/positivité des covariances historiques, mesure hors historique et purge de l'historique croisé.

## Points restant à traiter ou valider

1. **Horodatage lidar.** Le timestamp est celui fourni par la bibliothèque à la réception du point, pas nécessairement l'instant exact d'émission du rayon. `ClockAligner` recale l'origine des horloges, mais ne mesure ni la latence USB/série ni le délai propre au capteur. `lidar_latency_s` reste un paramètre de calibration. Une amélioration ultérieure serait d'estimer le timestamp de mesure à partir de la phase du tour et de sa durée, puis de comparer cette estimation aux timestamps de réception.
2. **Clusters traversant 0°/360°.** `Cluster.seam` signale un cluster fusionné à travers la coupure du tour. Ses points peuvent être séparés d'environ 100 ms; leur distance/angle moyens ne correspondent pas toujours à une mesure physique unique si le robot bouge. La correction de ces clusters doit être validée en mouvement; il peut être nécessaire de les écarter ou de les traiter séparément quand le déplacement pendant le tour dépasse un seuil.
3. **Points projetés avec un état `clamped`.** Le filtrage géométrique les conserve en utilisant l'état le plus proche. Il faut surveiller `points_unverified`; si ce compteur est non nul de façon récurrente, corriger la synchronisation ou rejeter ces points.
4. **Relocalisation désactivée.** Dans `core/localizer.py`, l'appel à `triangulate` est commenté et `found` est forcé à `None`. Le système peut donc déclarer la pose perdue, mais ne peut pas se relocaliser automatiquement dans cet état.
5. **Validation sur cible.** Les tests ajoutés couvrent la logique mathématique, mais doivent être exécutés depuis `lidar/` avec `python -m pytest tests/test_ekf_delayed_correction.py -v`, puis l'ensemble des tests. Ensuite, vérifier à l'arrêt et en mouvement : statut des associations (`matched/ambiguous/absent`), compteurs de corrections acceptées/rejetées, Mahalanobis, horodatages, et évolution des écarts-types.

## Diagnostic pratique si des poteaux sont visibles mais la pose ne bouge pas

La détection visuelle d'un cluster ne garantit pas qu'il soit associé ni accepté. Vérifier dans cet ordre :

1. le cluster a-t-il la taille d'un poteau (`n_size_ok`) ?
2. son association est-elle `matched`, ou `ambiguous` ?
3. `correct_pole` accepte-t-il la mesure ? Sinon, le Mahalanobis ou la limite de correction peut la rejeter;
4. l'horodatage du cluster tombe-t-il dans l'historique EKF et le retard `lidar_latency_s` est-il cohérent ?

Le fait que le robot soit à l'arrêt ne devrait pas, à lui seul, empêcher une correction : la décision dépend de l'association, du gating et des limites de correction, pas de la vitesse du robot.


## Ajustements de seuil et métrique de latence

- Le seuil de rejet chi² est passé de **9,21 à 18,42** (facteur 2). Il s'agit d'un filtre anti-aberrations moins restrictif; les garde-fous de déplacement maximal par correction restent actifs.
- `Report.acq_to_correction_ms` mesure la durée de bout en bout depuis le timestamp du premier point du tour (`rev.t_start_ns`) jusqu'à la fin du traitement et des corrections potentielles. Elle inclut donc la durée du tour lidar, l'attente éventuelle dans la file et le traitement logiciel. Elle est distincte de `proc_ms`, qui mesure seulement le temps de calcul.
- `Report.acq_to_correction_avg_5s_ms` est la moyenne glissante des durées de bout en bout des tours terminés au cours des cinq dernières secondes. Les deux métriques sont disponibles dans `Localizer.status()` et sont ajoutées au journal CSV (`acq_to_correction_ms`, `acq_to_correction_avg_5s_ms`).
