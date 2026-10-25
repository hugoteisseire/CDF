# Intégration de l'EKF sur le robot holonome : étapes

Oct 15, 2026 · @Someone

## Objectif et point de départ

L'EKF fusionne l'odométrie (continue, mais elle dérive) et les poteaux vus par le lidar (10 fois par seconde, mais sans dérive) pour donner une pose (x, y, θ) fiable en continu. L'intégration se fait en 8 étapes, de l'acquisition lidar jusqu'aux tests sur le robot, et chaque étape a un critère de passage avant de commencer la suivante.

Ce qu'on a déjà :

- Terrain 2 m × 2 m, 3 poteaux de 100 mm de diamètre aux coins (0, 0), (2, 0) et (0, 2) m.
- Robot holonome : 1 m/s et 150 °/s au maximum.
- Lidar RPLidar C1 : 10 Hz, ±30 mm, 0,7° de résolution.
- Un simulateur EKF pour tester les réglages sans robot.
- Un module d'acquisition lidar (thread dédié, capping, détection des clusters) qui fournit pour chaque tour la liste des clusters avec leur angle, leur distance et l'horodatage de chaque point.

Principe retenu : le robot ne cherche jamais « où il est » à partir de zéro. L'odométrie prédit où il est, le lidar vérifie et corrige, et un poteau qui ne colle pas à la prédiction est ignoré plutôt que cru.

## Étape 0 : acquisition lidar fiable à 10 Hz

L'EKF n'est branché que quand le lidar livre 10 tours par seconde, sans trou. Un tour manquant, c'est 100 mm de déplacement à 1 m/s sans correction ; des horodatages faux, c'est une correction appliquée au mauvais endroit.

À relever dans les statistiques du module, robot immobile puis en mouvement :

- fréquence de tours proche de 10 Hz, et tours publiés égaux aux tours vus (aucun tour rejeté) ;
- aucune erreur de la bibliothèque lidar sur plusieurs minutes ;
- aucun tour perdu avant traitement ;
- temps de traitement d'un tour nettement inférieur à 100 ms, avec une marge pour l'EKF ;
- les 3 poteaux détectés à chaque tour, robot immobile.

Critère de passage : ces points tenus pendant 5 minutes, y compris quand le reste du programme (EKF, asservissement, affichage) tourne en même temps. Si ce n'est pas tenu, on corrige l'acquisition avant tout le reste.

## Étape 1 : état, modèle et paramètres de l'EKF

L'EKF suit 3 grandeurs et, pour chacune, une incertitude (écart-type) que le filtre met à jour lui-même. Les valeurs de départ ci-dessous sont des points de départ à régler, pas des valeurs mesurées.

| Grandeur | Unité | Rôle | Valeur de départ (écart-type) |
| --- | --- | --- | --- |
| x, y | m | position du robot sur le terrain | 20 mm à la mise en route |
| θ | rad (°) | cap du robot | 1° à la mise en route |
| P | mm², rad² | incertitude actuelle de la pose | calculée par le filtre |
| Q | par pas d'odométrie | bruit de l'odométrie | voir étape 5 |
| R | mm², rad² | bruit de mesure d'un poteau | voir étape 5 |

Hypothèses à écrire noir sur blanc : repère terrain fixe avec l'origine sur le poteau (0, 0), angle lidar compté dans le sens horaire de 0 à 360°, repère robot en radians dans le sens trigonométrique, et lidar placé à l'origine du robot. Si le lidar est décalé du centre du robot, ce décalage (en mm) est à mesurer et à ajouter dans le modèle de mesure.

## Étape 2 : prédiction avec l'odométrie

À chaque pas d'odométrie, la pose avance du déplacement mesuré et l'incertitude grandit. C'est la seule chose que fait le filtre entre deux tours de lidar.

Exemple chiffré à vitesse maximale, avec un pas de 10 ms :

- 1 m/s × 10 ms = 10 mm de translation par pas ;
- 150 °/s × 10 ms = 1,5° de rotation par pas ;
- entre deux tours de lidar (100 ms), le robot a donc parcouru 100 mm et tourné de 15°.

On applique ce déplacement dans le repère du robot, converti dans le repère terrain avec le cap θ courant (c'est ce qui rend le filtre « étendu » : l'effet d'une erreur de cap dépend de la direction du mouvement). Chaque pas ajoute du bruit Q à l'incertitude : avec une odométrie à environ 2 % d'erreur, 1 m parcouru ajoute de l'ordre de 20 mm d'incertitude sur la position.

Point à ne pas oublier : le filtre garde l'historique des poses et des incertitudes sur au moins 1 tour (100 ms), pour pouvoir corriger au bon moment (étape 4).

## Étape 3 : association des clusters aux poteaux connus

Un vrai tour de lidar contient des dizaines de clusters (obstacles, adversaire, bords) pour 3 poteaux utiles. Le filtre ne cherche donc pas les poteaux « dans tout le tour » : il calcule où chaque poteau devrait apparaître et ne regarde que là.

1. Avec la pose prédite, calculer pour chaque poteau l'angle (°) et la distance (mm) attendus dans le repère du robot.
2. Ouvrir une fenêtre autour de cette position : par exemple ±3 ° et ±100 mm au départ, à resserrer quand l'incertitude P est faible et à élargir quand elle grandit.
3. Garder le cluster de la fenêtre dont la taille correspond à un poteau de 100 mm. S'il y en a deux, ne rien choisir et sauter ce poteau pour ce tour.
4. Un cluster hors de toute fenêtre est ignoré, jamais utilisé comme poteau.

Critère de passage : en simulation avec des obstacles parasites, chaque poteau est associé au bon cluster à chaque tour, sans association erronée.

## Étape 4 : correction séquentielle avec gating

Chaque poteau associé corrige la pose à son tour, avec l'instant exact de ses points : à 1 m/s, 100 ms d'écart entre le début et la fin d'un tour, c'est 100 mm de déplacement, bien plus que les ±30 mm du lidar.

Pour chaque poteau, dans l'ordre :

1. Prendre la pose prédite à l'instant du cluster (horodatage de ses points) et en déduire la mesure attendue : angle (°) et distance (mm).
2. Calculer l'écart entre la mesure réelle et la mesure attendue : écart en mm pour la distance, en ° pour l'angle (ramené entre −180° et +180°, sinon un poteau vers 0°/360° donne un écart de 360°).
3. Gating : comparer cet écart à ce que le filtre juge normal (incertitude de la prédiction + bruit de mesure). Si l'écart est trop grand, par exemple au-delà du seuil à 99 % pour 2 mesures, le poteau est rejeté pour ce tour et la pose ne bouge pas.
4. Sinon, corriger la pose : plus la prédiction est incertaine par rapport à la mesure, plus la correction est forte.
5. Passer au poteau suivant avec la pose déjà corrigée.

Critère de passage : en simulation, pose finale à moins de 10 mm de la vérité après quelques tours, et aucun saut de pose visible quand on corrige.

## Étape 5 : réglage de Q et R, avec unités

Q dit « combien je me fie peu à mon odométrie », R dit « combien je me fie peu à un poteau ». Le rapport entre les deux décide qui l'emporte dans la correction.

| Réglage | Unité | Valeur de départ | D'où elle vient |
| --- | --- | --- | --- |
| Bruit de distance d'un poteau (R, distance) | mm | 25 mm | ajustement du poteau testé avec un bruit de 30 mm sur les points |
| Bruit d'angle d'un poteau (R, angle) | ° | 0,4° | même test |
| Erreur d'odométrie en translation (Q) | % de la distance | 2 % | à mesurer sur le robot |
| Erreur d'odométrie en rotation (Q) | % de l'angle | 1 % | à mesurer sur le robot |

Méthode pour régler sans deviner : enregistrer, pour chaque correction acceptée, l'écart normalisé (l'écart divisé par ce que le filtre attendait). Pour 2 mesures, sa moyenne doit rester proche de 2. Bien au-dessus : Q ou R trop petits, le filtre est trop sûr de lui. Bien en dessous : trop gros, il corrige mollement.

Mesure de l'odométrie à faire une fois : aller-retour de 1 m répété 10 fois, et tour sur place, puis relever l'erreur réelle sur le terrain. C'est cette mesure qui remplace les 2 % et 1 % ci-dessus.

## Étape 6 : poteaux perdus, dérive d'odométrie, recalage

C'est la faille de la méthode de départ (« sans poteaux, on est perdu ») : l'EKF la traite par paliers, du plus doux au plus dur.

1. Un ou deux poteaux manquent : les autres corrigent normalement. Avec un seul poteau, la position est contrainte mais le cap l'est mal ; l'incertitude sur θ grandit plus vite.
2. Plus aucun poteau : seule l'odométrie avance, l'incertitude P grandit, et les fenêtres de l'étape 3 s'élargissent toutes seules.
3. Dérive d'odométrie (par exemple +20 cm constatés) : si les poteaux reviennent et que les écarts restent grands mais cohérents d'un tour à l'autre, on augmente Q temporairement pour que le lidar reprenne la main, sans saut brutal de la pose. On ne recale jamais d'un coup l'odométrie sur un seul tour de lidar : un tour aberrant déplacerait le robot à tort.
4. Poteaux rejetés pendant plus d'1 s (10 tours de suite) : le filtre se considère perdu. Il refait une localisation depuis zéro à partir de 2 poteaux (triangulation directe), réinitialise x, y, θ et P avec une grande incertitude, puis reprend le fonctionnement normal.

Dans tous les cas, l'incertitude P est publiée avec la pose : l'asservissement et la stratégie peuvent ralentir ou s'arrêter quand elle dépasse un seuil (par exemple 50 mm).

## Étape 7 : tests de validation et ordre de mise en service

On avance de test en test, de la simulation vers le robot ; on ne passe au suivant que si le précédent est propre.

1. Simulateur EKF : régler Q et R, vérifier l'association et le gating avec des obstacles parasites.
2. Robot immobile : pose stable à quelques mm près pendant 1 minute, incertitude P qui se stabilise.
3. Rotation sur place à 150 °/s : le cap suit sans décalage, les 3 poteaux restent associés.
4. Translation lente puis à 1 m/s : pas de saut de pose, écart normalisé moyen proche de 2.
5. Poteau masqué à la main : la pose reste correcte avec 2 puis 1 poteau, P grandit puis se resserre quand il revient.
6. Odométrie faussée volontairement de +20 cm : le filtre se resserre sur la vraie position en quelques tours, sans saut.
7. Matches complets en conditions réelles, avec journal de la pose, de P et de l'écart normalisé pour analyser après coup.

## Points ouverts et décisions à prendre

- Le module lidar actuel doit encore passer le critère de l'étape 0 sur le robot : le test en simulation avec livraison par rafales n'a pas encore été mené à terme.
- Les bords du terrain sont-ils à hauteur du lidar ? Si oui, un poteau au coin peut fusionner avec le bord et ne plus être détecté comme cluster isolé.
- Le lidar est-il bien au centre du robot ? Sinon, mesurer le décalage (mm).
- Cadence réelle de l'odométrie (le pas de 10 ms ci-dessus est un exemple).
- Valeurs réelles de Q, mesurées sur le robot (étape 5).
- Seuils de perte (1 s) et d'arrêt de sécurité (P > 50 mm) à confirmer avec la stratégie de jeu.
