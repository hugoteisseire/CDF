# base_holo

Contrôle de la base holonome 3 roues (servos MKS sur bus CAN, socketcan 1 Mbit/s).

```
core/       holo_base.py       cinématique (Robot, distances / vitesses de roue)
            motor_controller.py  Motor (protocole MKS) et MotorGroup (mouvements, watchdog, poller)
            holonomic_base.py  HolonomicBase : façade au-dessus de Robot + MotorGroup, odométrie
examples/   teleop_zqsd.py     pilotage au clavier + affichage de la pose
tests/      test_odometry.py        validation / recalage des paramètres du robot (à relancer
                                    quand la mécanique change : signe, rayon, diamètre de roue)
            test_velocity_square.py carré en mode vitesse (MotorGroup.move_velocity_direction)
            test_wheel_distances.py calcul pur, sans matériel
tools/      calculate_motor_params.py, simulation_traj.py (simulation pygame)
legacy/     ancienne architecture et exemples abandonnés (non maintenus)
```

Lancer depuis ce dossier, en mode module :

```
python -m tests.test_wheel_distances      # sans matériel
python -m tests.test_odometry             # robot, bus CAN actif
python -m tests.test_velocity_square
python -m examples.teleop_zqsd
```

Après un changement mécanique : relancer `tests/test_odometry` (signe de position, rayon,
diamètre de roue) puis mettre à jour `wheel_diameter_mm`, `gear_ratio` et `radius` dans les scripts.
