# Guide de reprise — visualiseur de table (`viz_table`)

> Document de référence pour reprendre le développement du visualiseur dans une future discussion.
> Périmètre : dossier `viz_table/` du dépôt `hugoteisseire/CDF`. Les détails ci-dessous sont basés sur les fichiers du dépôt consultés le 22 octobre 2026.

## 1. Objectif général

`viz_table` est un visualiseur web de la table de jeu, prévu notamment pour la Coupe de France de Robotique. Il dessine dans un canvas le robot, l'adversaire, les éléments de jeu et des formes libres. Plusieurs programmes Python peuvent publier des données indépendamment ; le visualiseur centralise ces données et permet aussi d'envoyer des commandes aux programmes.

Le visualiseur est un outil d'observation et d'interaction. Il ne calcule pas lui-même la localisation réelle, la trajectoire ou la stratégie : ces fonctions restent dans les programmes qui publient leurs résultats.

## 2. Architecture

Trois composants principaux communiquent par WebSocket :

1. **Page web — `index.html`**
   - Dessine la table et les couches reçues dans un canvas.
   - Affiche l'état du serveur, les programmes connectés, les informations du robot et les couches disponibles.
   - Permet de masquer des couches, d'inverser l'affichage gauche/droite, d'afficher une grille et d'effacer la traînée du robot.
   - Propose des boutons et des modes d'interaction (placer/déplacer des éléments, placer le robot ou l'adversaire, envoyer un `goto`, etc.).
   - Se connecte à `/ws/view` et se reconnecte après une coupure.

2. **Serveur central — `server.py`**
   - Application FastAPI servie par Uvicorn sur le port TCP **8000**, à l'écoute sur `0.0.0.0`.
   - Sert `index.html`, `config.js` et éventuellement `table.png`.
   - Accepte les programmes sur `/ws/prog?name=<nom>` et le navigateur sur `/ws/view`.
   - Garde uniquement la **dernière valeur** de chaque couche dans le dictionnaire `layers` : ce n'est pas un historique.
   - Diffuse les mises à jour aux navigateurs et relaie les commandes de la page aux programmes abonnés.
   - Expose `/state` pour inspecter l'état courant en JSON.
   - Actualise environ chaque seconde l'état de connexion des programmes.

3. **Client Python — `viz_client.py`**
   - À importer par chaque programme qui veut publier des données ou recevoir des commandes.
   - La classe `Viz` ouvre une connexion WebSocket dans un thread de fond et tente automatiquement de se reconnecter après une coupure.
   - `send()` et `clear()` ne doivent pas bloquer la boucle principale.
   - Les callbacks enregistrés avec `on()` s'exécutent dans le thread de réception : ils doivent être courts. Pour un travail plus long, le callback devrait simplement poser un drapeau ou mettre une tâche dans une file traitée par la boucle principale.

Schéma logique :

```text
Programmes Python (robot / localisation / stratégie / lidar...)
       |  Viz.send(...) et Viz.on(...)
       v
  /ws/prog — server.py — état courant des couches
       |                         ^
       | diffusion des couches   | commandes de l'interface
       v                         |
Navigateur : index.html <--- /ws/view
       |
       +-- affichage canvas
       +-- interactions utilisateur
```

## 3. Installation et lancement

Sur la Raspberry Pi ou la machine qui héberge le serveur :

```bash
cd CDF/viz_table
python -m pip install -r requirements.txt
python server.py
```

Dépendances déclarées dans `requirements.txt` :
- `fastapi`
- `uvicorn[standard]`
- `websocket-client`

Depuis un navigateur connecté au même réseau, ouvrir :

- `http://<ip-de-la-pi>:8000` : interface graphique.
- `http://<ip-de-la-pi>:8000/state` : état brut des couches et statut des clients.

Lancer ensuite les programmes Python qui alimentent le visualiseur. Pour valider l'installation, deux programmes d'exemple sont disponibles :

```bash
python example_robot.py
python example_strategie.py
```

Ils sont des démonstrations/simulations, pas les pilotes du vrai robot. Il faut que le serveur soit déjà démarré. Si l'accès depuis un autre appareil échoue, vérifier l'adresse IP, le réseau et l'autorisation du port 8000 dans le pare-feu.

## 4. Utilisation du client dans un programme

Exemple minimal :

```python
from viz_client import Viz

viz = Viz("localisation", host="127.0.0.1", port=8000)

# Publier la pose du robot : x/y en mm, theta en degrés
viz.send("robot", {
    "x": 1200,
    "y": 800,
    "theta": 90,
    "v": 0.0,
    "omega": 0.0,
    "r": 150,
    "target": None,
})

# Recevoir une commande envoyée depuis la page
def on_reset(args):
    # Callback court : poser un drapeau à traiter dans la boucle principale
    print("Demande de remise à zéro", args)

viz.on("reset_position", on_reset)
```

Points importants :
- Le nom passé à `Viz("localisation")` identifie le programme dans la liste des clients. Deux connexions portant le même nom se remplacent côté serveur : choisir des noms distincts.
- Le client utilise `127.0.0.1` par défaut. Si le programme tourne sur un autre ordinateur que le serveur, passer l'adresse IP de la machine hébergeant le serveur, par exemple `Viz("localisation", host="192.168.1.20")`.
- `send(layer, data)` remplace les données de cette couche ; il ne fusionne pas automatiquement les dictionnaires.
- `clear(layer)` supprime une couche.
- Les données doivent être sérialisables en JSON : nombres, chaînes, booléens, listes, dictionnaires ou `None`.
- L'envoi peut échouer si le serveur est absent ou si la connexion n'est pas encore établie. Le client retente la connexion, mais il ne met pas les messages d'envoi en attente : republier périodiquement l'état courant est donc le bon modèle.
- `viz.on("commande", callback)` abonne le programme à une commande précise. `viz.on("*", callback)` permet de toutes les recevoir ; dans ce cas le callback reçoit `(args, cmd)`.

## 5. Repère, unités et dimensions

Le repère utilisé par l'interface et les données est celui de la table :

- Dimensions par défaut : **3000 × 2000 mm**.
- Origine `(0, 0)) : coin inférieur gauche.
- Axe `x) : vers la droite.
- Axe `y) : vers le haut.
- Angles `theta) : degrés, `0°) dans le sens de `x+`, sens trigonométrique positif.
- Les positions et dimensions géométriques sont en millimètres.
- La vitesse `v) affichée est en m/s ; `omega) est en degrés/s.

L'interface convertit le repère de la table en pixels du canvas. La case « Inverser gauche/droite » inverse la représentation horizontale pour un changement de côté ; elle ne doit pas être interprétée comme une transformation appliquée aux données publiées par les programmes.

## 6. Couches affichées

Une couche est identifiée par un nom et contient les dernières données envoyées par un programme. Les noms sont libres : une nouvelle couche apparaît automatiquement dans la liste « Couches affichées ».

### Couche `robot`

Format courant :

```json
{
  "x": 1200,
  "y": 800,
  "theta": 90,
  "v": 0.3,
  "omega": 0,
  "r": 150,
  "target": [1500, 1000]
}
```

- `x`, `y) : position du centre du robot, en mm.
- `theta) : orientation, en degrés.
- `v) : vitesse affichée en m/s.
- `omega) : vitesse angulaire affichée en degrés/s.
- `r) : rayon dessiné, 150 mm par défaut.
- `target) : cible facultative sous la forme `[x, y]`, ou `null`.

La page met à jour le panneau d'informations et dessine une traînée à partir des positions successives. La traînée est maintenue côté navigateur, avec une limite de 600 points ; elle n'est pas envoyée par le client Python.

### Couche `opponent`

Format usuel : `{"x": 2200, "y": 1300, "rmax": 250}`.

- `x), `y) : position de l'adversaire en mm.
- `rmax) : rayon ou marge de représentation selon le dessin prévu par l'interface.
- L'interface indique l'ancienneté de la dernière donnée reçue.

### Couche d'éléments de jeu

N'importe quelle couche contenant une clé `items` peut représenter une collection d'éléments. Exemple utilisé dans la stratégie :

```json
{
  "items": [
    {
      "id": "e0",
      "x": 760,
      "y": 55,
      "theta": 0,
      "value": 3,
      "color": "#d4a017"
    }
  ]
}
```

Chaque élément possède généralement :
- `id) : identifiant stable utilisé pour déplacer, orienter ou supprimer l'élément.
- `x), `y) : centre de l'élément en mm.
- `theta) : angle en degrés. À `0°`, le grand côté est aligné sur l'axe `x`.
- `value) : entier affiché au centre ; les valeurs de 0 à 3 sont usuelles. La valeur `-1` sélectionne la géométrie courte spéciale.
- `w), `l) : dimensions optionnelles.
- `color) : couleur optionnelle.

Dimensions par défaut : élément standard de **110 × 320 mm** ; élément de valeur `-1` de **110 × 100 mm**. Les dimensions peuvent être personnalisées dans `config.js`.

### Couche `shapes) et formes libres

Une couche contenant `shapes` permet d'afficher des annotations, zones ou géométries de debug sans modifier le serveur. Les types documentés dans le README sont :

- `circle` : `{x, y, r}`
- `rect` : `{x, y, w, h}`, avec `x,y` au coin inférieur gauche
- `line` : `{x1, y1, x2, y2}`
- `polyline` : `{points: [...]} `
- `text` : `{x, y, text}`

Les propriétés `color` et `fill` sont facultatives. Cela peut servir à visualiser une cible, des waypoints, une trajectoire calculée, une zone de sécurité, des obstacles du lidar ou des résultats d'algorithmes. Vérifier le format exact attendu par le code de dessin avant d'ajouter un nouveau type de forme.

## 7. Commandes de l'interface vers les programmes

Les boutons et les interactions du canvas envoient une commande sous la forme logique `{type: "cmd", cmd: "...", args: {...}}`. Le serveur ne l'exécute pas : il la transmet aux programmes abonnés.

Commandes documentées :

| Commande | Arguments principaux | Utilisation |
|---|---|---|
| `estop` | aucun | Demande d'arrêt logiciel |
| `reset_table` | aucun | Réinitialisation des éléments de table |
| `reset_position` | aucun | Demande de remise à zéro de la pose |
| `start_strategy` | aucun | Demande de lancement de stratégie |
| `stop_strategy` | aucun | Demande d'arrêt de stratégie |
| `set_strategy` | `{name}` | Sélection d'une stratégie |
| `goto` | `{x, y}` | Demande d'aller vers un point |
| `set_robot_pose` | `{x, y}`, éventuellement `theta` | Placement manuel du robot |
| `set_opponent` | `{x, y}` | Placement de l'adversaire |
| `add_element` | `{x, y}`, éventuellement `theta` | Ajout d'un élément |
| `move_element` | `{id, x, y}` | Déplacement d'un élément |
| `rotate_element` | `{id, theta}` | Rotation d'un élément |
| `remove_element` | `{id}` | Suppression d'un élément |

Les commandes ne sont reçues que si un programme s'y est abonné avec `viz.on(...)`. Si aucun programme connecté n'écoute une commande, la page affiche un avertissement. Le serveur ne valide pas que l'action est sûre ni qu'elle a été exécutée.

**Sécurité :** le bouton « ARRÊT D'URGENCE » passe par le Wi-Fi, le navigateur et les programmes. Il ne remplace absolument pas un arrêt d'urgence matériel indépendant.

## 8. Configuration et personnalisation

Le fichier `index.html` contient les valeurs par défaut. Pour les personnaliser durablement :

```bash
cp config.example.js config.js
```

Modifier ensuite `config.js`, puis recharger la page avec `Ctrl+F5`. Le fichier personnel `config.js) et l'image `table.png) ne sont pas inclus dans l'archive de mise à jour afin de ne pas écraser les réglages personnels.

Paramètres principaux exposés dans `window.CONFIG` :
- `W`, `H`, `M) : dimensions de la table et marge d'affichage.
- `ELEM_W`, `ELEM_L), `ELEM_L_M1) : dimensions des éléments.
- `TABLE_IMAGE) : chemin de l'image de fond, ou chaîne vide pour ne pas en afficher.
- `ROBOT_FILL), `ROBOT_STROKE) : couleurs du robot.
- `BUTTONS) : boutons et commandes envoyées.
- `STRATEGIES) : options du sélecteur de stratégie.
- `MODES) : commandes disponibles dans la section d'interaction au clic.

L'image `table.png` se place à côté de `server.py`. Elle doit avoir un ratio 3:2, être rognée aux bords du terrain et correspondre à l'origine en bas à gauche. Si l'affichage est inversé, l'image l'est également.

## 9. Rôle des fichiers du dossier

| Fichier | Rôle |
|---|---|
| `server.py` | Serveur HTTP/WebSocket, état courant des couches, relais des commandes |
| `index.html` | Interface graphique, canvas, dessin, interactions, configuration par défaut |
| `viz_client.py` | API Python commune pour publier des couches et écouter les commandes |
| `config.example.js` | Exemple de configuration personnelle |
| `requirements.txt` | Dépendances Python |
| `example_robot.py` | Robot et adversaire simulés ; exemple de publication et de réception des commandes |
| `example_strategie.py` | Éléments de jeu simulés, gestion de leur ajout/déplacement/retrait, zones et alimentation simulée de la pose |
| `README.md` | Installation et résumé du protocole et des conventions |
| `config.js` | Configuration locale à créer ; peut être absent du dépôt |
| `table.png` | Image de fond locale facultative |

## 10. Points de vigilance et limites actuelles

1. **Pas d'historique serveur :** chaque couche ne conserve que sa dernière valeur. Pour enregistrer des scans lidar ou analyser un historique, il faut une autre solution.
2. **La réception ne garantit pas la fraîcheur des données :** le navigateur conserve la dernière valeur reçue jusqu'à un nouvel envoi ou un effacement. Le timestamp côté page correspond à la réception, pas nécessairement à l'instant de mesure du capteur.
3. **Reconnexion et perte de messages :** le client Python se reconnecte, mais ne met pas les envois en file d'attente pendant une déconnexion. Publier régulièrement l'état courant aide à rétablir l'affichage.
4. **Callbacks dans un thread de réception :** éviter les attentes longues, les calculs coûteux et les accès concurrents non protégés dans les callbacks. Utiliser des drapeaux, une file de messages ou un verrou adapté.
5. **Commandes sans accusé d'exécution :** le relais indique essentiellement si un programme écoute ; cela ne prouve pas que la commande a réussi. Ajouter un mécanisme d'accusé de réception si cette distinction devient importante.
6. **Arrêt d'urgence :** le bouton logiciel n'est pas un dispositif de sécurité matériel.
7. **Simulation et robot réel :** `example_robot.py` déplace un robot fictif. `example_strategie.py` lit `/state` pour simuler la réception de la pose et de l'adversaire ; ce fil de simulation doit être remplacé par des appels aux sources réelles dans le programme final.
8. **Réseau :** le serveur écoute sur toutes les interfaces. Ne pas exposer le port 8000 à un réseau non fiable sans ajouter des protections adaptées.

## 11. Méthode recommandée pour ajouter une nouvelle visualisation

1. Identifier quel programme possède réellement la donnée (localisation, lidar, planification, stratégie).
2. Définir un format de données stable, en précisant unités, repère, angle et cadence de mise à jour.
3. Publier les données via `viz.send("nom_couche", donnees)`.
4. Pour un simple overlay, préférer une couche `shapes` si les primitives existantes suffisent : cela évite de modifier `server.py` et souvent `index.html`.
5. Si un nouveau type de données nécessite un dessin spécialisé, modifier le rendu dans `index.html` et documenter le schéma ici.
6. Si l'utilisateur doit commander le programme, ajouter une commande à l'interface/configuration et s'abonner dans le programme avec `viz.on(...)`.
7. Tester d'abord avec des données simulées, puis avec le programme réel. Vérifier `/state`, le statut du client et les unités affichées.

## 12. Contexte à réutiliser dans une prochaine discussion

Pour reprendre le travail, fournir ce document ou indiquer que le projet concerné est `hugoteisseire/CDF/viz_table`. Préciser ensuite le fichier ou le comportement visé, ainsi que le résultat attendu.

Repères utiles :
- Le visualiseur est un outil d'affichage et de commande, pas le cœur de la stratégie ou de la localisation.
- Le protocole existant est volontairement générique : **couches JSON en sortie des programmes, commandes JSON en retour**.
- Pour visualiser une trajectoire, des waypoints ou des zones calculées, commencer par une couche `shapes`.
- Pour déboguer la communication, vérifier le serveur sur le port 8000, la connexion du client, puis `/state`.
- Les données géométriques doivent rester cohérentes avec le repère de la table : origine en bas à gauche, millimètres et degrés.
