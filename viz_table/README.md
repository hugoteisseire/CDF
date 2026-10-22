# Visualiseur de table de jeu

Une page web (canvas) affiche le robot, l'adversaire et les éléments de jeu.
Plusieurs programmes Python y envoient leurs données via un petit serveur central.

## Installation (sur la Raspberry Pi)

    pip install -r requirements.txt

## Lancement

    python server.py              # serveur du visualiseur, port 8000
    python example_robot.py       # programme robot simulé + serveur de protocole local
    python example_strategie.py   # stratégie + client de protocole local

Ouvrir `http://<ip-de-la-pi>:8000` depuis n'importe quel appareil du réseau.

Le protocole stratégie ↔ robot est documenté dans [ROBOT_PROTOCOL.md](ROBOT_PROTOCOL.md).
Il utilise une socket Unix locale, indépendamment du serveur web. La stratégie reçoit l'état
à 50 Hz et peut appeler `robot_link.goto(...)`, `robot_link.velocity(...)`,
`robot_link.reset_pose(...)` ou `robot_link.stop()`. En cas de déconnexion stratégie,
le simulateur arrête le mouvement ; à la reconnexion, il reste arrêté jusqu'à une nouvelle commande.
Le reset de pose n'est pas exposé à l'afficheur.
`http://<ip-de-la-pi>:8000/state` donne l'état brut des couches (utile pour déboguer).

## Utiliser l'affichage depuis un de vos programmes

    from viz_client import Viz
    viz = Viz("localisation")                                 # nom du programme (pastille verte/rouge)
    viz.send("robot", {"x": 1200, "y": 800, "theta": 90})     # mm, degrés
    viz.on("reset_position", lambda args: ...)                # commande envoyée par la page

Copier `viz_client.py` à côté de vos programmes ; dépendance : `websocket-client`.
`send()` ne bloque jamais, la reconnexion au serveur est automatique.
Les fonctions passées à `on()` doivent être courtes.

## Conventions

- Repère : origine (0, 0) en bas à gauche de la table, x vers la droite, y vers le haut.
- Unités : mm, degrés (0° vers x+, sens trigonométrique), m/s, m/s².
- Table : 3000 x 2000 mm (réglable dans `config.js`, voir plus bas).

## Réglages personnels et image de fond

Les valeurs par défaut sont dans `index.html`. Pour les changer sans les perdre à la prochaine mise à jour :
copier `config.example.js` en `config.js` (à côté de `server.py`) et décommenter ce qui doit changer
(taille de la table, dimensions des éléments, couleur du robot, boutons, stratégies...).

Image de fond : placer `table.png` à côté de `server.py` (ratio 3:2, rognée aux bords du terrain,
origine en bas à gauche). Sans image, la page affiche un fond uni. Si l'affichage est inversé
(changement de côté), l'image est retournée aussi.

`config.js` et `table.png` ne sont jamais dans l'archive : une mise à jour ne les écrase pas.
Après toute modification, recharger la page (Ctrl+F5).

## Couches reconnues par la page

| Couche (nom) | Contenu |
|---|---|
| `robot` | `{x, y, theta, v, omega, r, target: [x, y]}` ; `r` = rayon dessiné (150 par défaut), la traînée est tracée par la page |
| `opponent` | `{x, y, rmax}` ; la page affiche « vu il y a … s » si la donnée vieillit |
| toute couche avec `items` | éléments de jeu : `{"items": [{id, x, y, theta, value, w?, l?, color?}]}`. `value` : entier de 0 à 3 affiché au centre de l'élément ; la valeur -1 donne un élément de 110 × 100 mm (`ELEM_L_M1`). Rectangle de 110 × 320 mm par défaut (`w` = petit côté, `l` = grand côté ; réglables dans `config.js`). `theta` en degrés : 0° = grand côté le long de x ; un trait part du centre vers l'extrémité côté `theta` |
| toute couche avec `shapes` | formes libres : `circle {x,y,r}`, `rect {x,y,w,h}` (coin bas-gauche), `line {x1,y1,x2,y2}`, `polyline {points}`, `text {x,y,text}` ; `color`/`fill` optionnels |

Un nom de couche nouveau apparaît tout seul dans la liste « Couches affichées » (case à cocher).
Le chemin planifié, les waypoints ou toute autre future donnée peuvent donc passer par une couche `shapes`, sans toucher au serveur.

## Commandes envoyées par la page

Un programme reçoit une commande seulement s'il l'a demandée avec `viz.on("<commande>", fonction)`
(ou `viz.on("*", fonction)` pour tout recevoir). Si personne ne l'écoute, la page affiche un avertissement.

| Commande | Arguments |
|---|---|
| `estop`, `reset_table`, `reset_position`, `start_strategy`, `stop_strategy` | aucun |
| `set_strategy` | `{name}` |
| `goto` | `{x, y}` |
| `set_robot_pose` | `{x, y}` et `theta` si l'utilisateur a glissé |
| `set_opponent` | `{x, y}` |
| `add_element` | `{x, y}` et `theta` si l'utilisateur a glissé (la valeur initiale est choisie par la stratégie) |
| `move_element` | `{id, x, y}` |
| `rotate_element` | `{id, theta}` |
| `remove_element` | `{id}` |

Pour ajouter un bouton : une ligne dans `BUTTONS` de `config.js`.

Attention : le bouton « ARRÊT D'URGENCE » passe par le Wi-Fi et par vos programmes ; il ne remplace pas un arrêt d'urgence matériel.
