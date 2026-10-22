# Protocole robot local — version 1

Ce document décrit les échanges entre la stratégie et le programme robot (réel ou simulateur). Le visualiseur reste un canal séparé.

## Transport

- Socket Unix AF_UNIX/SOCK_STREAM, chemin par défaut /tmp/cdf_robot.sock.
- Le chemin peut être remplacé par la variable d'environnement CDF_ROBOT_SOCKET.
- Encodage UTF-8, une trame JSON par ligne (JSON Lines).
- Le serveur est le programme robot ; le client est la stratégie.
- Une seule connexion stratégie active est servie à la fois.
- La première trame est {"v":1,"type":"hello","source":"strategy"}.
- La connexion est considérée perdue si le socket se ferme ou si le processus stratégie s'arrête. Le robot doit immédiatement arrêter le mouvement. Les commandes ne sont jamais mises en attente pendant une déconnexion.

## Conventions géométriques

- Position : millimètres ; accélération : mm/s² ; vitesse linéaire : mm/s.
- Angles : degrés, 0° sur X+, sens trigonométrique positif.
- Repère terrain field : X vers la droite de la table, Y vers le haut.
- Repère robot : X vers l'avant du robot, Y vers sa gauche.
- theta de l'état est l'orientation du robot par rapport au repère terrain.
- vx/vy dans l'état sont exprimés dans le repère terrain ; omega en degrés/s.

## Enveloppe

Toutes les trames contiennent v: 1 et type. Les commandes contiennent un entier id croissant côté client. Le serveur répond par ack pour chaque commande reçue. Les trames state contiennent un compteur seq et un horodatage monotone timestamp_ns.

## Commandes stratégie → robot

### Vitesse vectorielle — cmd.velocity

    {"v":1,"type":"cmd.velocity","id":1,"frame":"robot","vx":200,"vy":0,"omega":0}

vx/vy sont en mm/s, omega en degrés/s. frame vaut robot ou field. La commande reste active jusqu'à une nouvelle commande, un arrêt ou une déconnexion.

### Déplacement — cmd.goto

    {"v":1,"type":"cmd.goto","id":2,"x":1500,"y":1000,"theta":90,"speed":500,"acc":800}

x/y et l'angle theta optionnel définissent la cible dans le repère terrain. speed est la vitesse linéaire maximale en mm/s ; acc est l'accélération linéaire maximale en mm/s². Une nouvelle cible remplace la précédente.

### Réinitialisation de pose — cmd.reset_pose

    {"v":1,"type":"cmd.reset_pose","id":3,"x":500,"y":300,"theta":0}

Cette commande réinitialise directement la pose/localisation. Elle n'est acceptée que sur la connexion identifiée comme stratégie. Elle ne doit pas être proposée par l'interface viz_table. Le programme robot arrête le mouvement avant de l'appliquer.

### Arrêt — cmd.stop

    {"v":1,"type":"cmd.stop","id":4}

Interrompt le mouvement en cours. L'arrêt logiciel ne remplace pas l'arrêt d'urgence matériel.

## État robot → stratégie

Le programme robot émet un state toutes les 20 ms (50 Hz), avec le dernier état disponible :

    {
      "v": 1,
      "type": "state",
      "seq": 123,
      "timestamp_ns": 123456789,
      "robot": {"x": 1200, "y": 800, "theta": 45, "vx": 200, "vy": 0, "omega": 0},
      "opponent": {"valid": true, "x": 2100, "y": 900, "theta": 180},
      "motion": {"mode": "goto", "target_x": 1500, "target_y": 1000,
                 "target_theta": 90, "moving": true}
    }

opponent.valid=false signifie que la position de l'adversaire n'est pas disponible ou n'est plus fiable. Les coordonnées d'adversaire invalides ne doivent pas être exploitées.

## Acquittements

Acceptation :

    {"v":1,"type":"ack","id":2,"status":"accepted"}

Refus :

    {"v":1,"type":"ack","id":3,"status":"rejected","reason":"motion_active"}

accepted signifie que la commande a été validée et mise en file de traitement, pas que le déplacement est terminé. La fin du goto se lit dans state.motion.moving=false (et mode=idle).

## Règles de sécurité et de fraîcheur

1. Si la connexion stratégie est perdue, le robot arrête le mouvement et annule la cible.
2. À la reconnexion, le robot reste arrêté jusqu'à une nouvelle commande explicite.
3. Aucun message n'est mis en file côté client durant une déconnexion.
4. Les trames mal formées, la version inconnue, les champs non numériques/non finis et les vitesses ou accélérations non positives pour un goto sont rejetés.
5. La pose ne peut être réinitialisée que par la stratégie. Le champ source ne suffit pas à lui seul : le serveur vérifie aussi l'identification initiale de la connexion.
6. Les sockets Unix sont locales à la machine ; le fichier de socket est créé avec les permissions 0600.
7. L'arrêt en cas de déconnexion est une sécurité logicielle ; il ne remplace pas l'arrêt d'urgence matériel.

## API Python

Dans la stratégie :

    from robot_protocol import RobotProtocolClient

    robot = RobotProtocolClient()
    robot.goto(1500, 1000, speed=500, acc=800, theta=90)
    robot.velocity(200, 0, 0, frame="robot")
    robot.reset_pose(500, 300, 0)  # stratégie uniquement
    robot.stop()
    state = robot.get_state()      # dernière trame, sans attente réseau
    robot.close()

Dans le programme robot ou simulateur :

    from robot_protocol import RobotProtocolServer

    server = RobotProtocolServer(state_provider).start()
    event = server.poll(timeout=0.0)  # commande ou événement de connexion
    server.close()

RobotProtocolServer ne pilote pas lui-même les moteurs : le programme consommateur applique les commandes à sa boucle de contrôle. Garder cette boucle indépendante du réseau.
