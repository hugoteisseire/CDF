# base_holo

Contrôle de la base holonome 3 roues (servos MKS sur bus CAN).

```
core/       holo_base.py (cinématique), motor_controller.py (classes Motor / MotorGroup)
legacy/     ancienne architecture (lib_moteur.py, test_base_holo.py)
examples/   exemples d'utilisation
tests/      tests (test_wheel_distances = calcul pur ; les autres exigent le bus CAN)
tools/      calculate_motor_params.py, simulation_traj.py (simulation pygame)
docs/       CHANGELOG et README du mode asynchrone
```

Lancer les scripts depuis ce dossier, en mode module :

```
python -m tests.test_wheel_distances
python -m examples.example_robot_movements
```
