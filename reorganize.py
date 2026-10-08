#!/usr/bin/env python3
"""
Réorganise simu/base_holo-prog/base_holo-prog  ->  simu/base_holo

Usage (depuis la racine du dépôt CDF) :
    python reorganize.py --dry-run    # affiche ce qui sera fait, sans rien modifier
    python reorganize.py              # exécute la réorganisation

Ce que fait le script :
  1. git mv de chaque fichier vers core/ legacy/ examples/ tests/ tools/ docs/
  2. réécrit les imports (holo_base, motor_controller, lib_moteur -> paquets)
  3. supprime l'import parasite `from turtle import speed`
  4. retire du suivi git les fichiers __pycache__/*.pyc et crée/complète .gitignore
  5. ajoute des __init__.py et un README.md décrivant la structure

Les valeurs par défaut de Motor (58 mm, 15/48) ne sont PAS modifiées.
"""

import re
import shutil
import subprocess
import sys
from pathlib import Path

OLD = Path("simu/base_holo-prog/base_holo-prog")
NEW = Path("simu/base_holo")

MOVES = {
    "core": ["holo_base.py", "motor_controller.py"],
    "legacy": ["lib_moteur.py", "test_base_holo.py"],
    "examples": ["example_motor_class.py", "example_robot_movements.py",
                 "example_wheel_distances.py"],
    "tests": ["test_mot.py", "test_move_distance.py", "test_wheel_distances.py"],
    "tools": ["calculate_motor_params.py", "simulation_traj.py"],
    "docs": ["CHANGELOG_MOVE_DISTANCE.md", "README_MOTOR_ASYNC.md"],
}

IMPORT_RULES = [
    (re.compile(r"^(\s*)from holo_base import", re.M), r"\1from core.holo_base import"),
    (re.compile(r"^(\s*)from motor_controller import", re.M), r"\1from core.motor_controller import"),
    (re.compile(r"^(\s*)from lib_moteur import", re.M), r"\1from legacy.lib_moteur import"),
    (re.compile(r"^(\s*)import holo_base\b", re.M), r"\1import core.holo_base as holo_base"),
    (re.compile(r"^(\s*)import motor_controller\b", re.M), r"\1import core.motor_controller as motor_controller"),
    (re.compile(r"^(\s*)import lib_moteur\b", re.M), r"\1import legacy.lib_moteur as lib_moteur"),
    # import parasite (autocomplétion) : inutile et plante sans Tk
    (re.compile(r"^from turtle import speed[ \t]*\n", re.M), ""),
]

AVOIDANCE_NOTE = (
    "# TODO: avoidance.py se trouve dans simu/simu_traj/ (pas dans ce dossier).\n"
    "#       Cet import ne fonctionne pas tant qu'il n'est pas rendu accessible.\n"
)
AVOIDANCE_RE = re.compile(r"^(from avoidance import)", re.M)

GITIGNORE_LINES = ["__pycache__/", "*.pyc", ".venv/", "venv/"]

README = """# base_holo

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
"""


def run(cmd, dry):
    print("  $", " ".join(cmd))
    if not dry:
        subprocess.run(cmd, check=True)


def git_out(*args):
    return subprocess.run(["git", *args], check=True, capture_output=True,
                          text=True).stdout


def main():
    dry = "--dry-run" in sys.argv
    if not Path(".git").exists():
        sys.exit("Lance ce script depuis la racine du dépôt (là où se trouve .git).")
    if not OLD.exists():
        sys.exit(f"Dossier introuvable : {OLD}")
    if NEW.exists():
        sys.exit(f"{NEW} existe déjà : abandon pour ne rien écraser.")
    if git_out("status", "--porcelain").strip():
        sys.exit("Le dépôt a des modifications non commitées. Commit ou stash d'abord.")

    print("== 1. Déplacement des fichiers (git mv)")
    for sub, files in MOVES.items():
        if not dry:
            (NEW / sub).mkdir(parents=True, exist_ok=True)
        for f in files:
            src = OLD / f
            if not src.exists():
                print(f"  (absent, ignoré) {src}")
                continue
            run(["git", "mv", str(src), str(NEW / sub / f)], dry)

    print("== 2. Réécriture des imports")
    if not dry:
        for py in NEW.rglob("*.py"):
            text = py.read_text(encoding="utf-8")
            new = text
            for rx, repl in IMPORT_RULES:
                new = rx.sub(repl, new)
            if AVOIDANCE_RE.search(new) and "avoidance.py se trouve" not in new:
                new = AVOIDANCE_RE.sub(lambda m: AVOIDANCE_NOTE + m.group(1), new, count=1)
            if new != text:
                py.write_text(new, encoding="utf-8")
                print(f"  modifié : {py}")
    else:
        print("  (dry-run : imports non modifiés)")

    print("== 3. __init__.py et README")
    if not dry:
        for sub in MOVES:
            if sub != "docs":
                (NEW / sub / "__init__.py").touch()
        (NEW / "README.md").write_text(README, encoding="utf-8")

    print("== 4. Fichiers compilés (.pyc) et .gitignore")
    tracked_pyc = [p for p in git_out("ls-files").splitlines() if p.endswith(".pyc")]
    for p in tracked_pyc:
        run(["git", "rm", "--cached", "-q", p], dry)
    gi = Path(".gitignore")
    existing = gi.read_text(encoding="utf-8").splitlines() if gi.exists() else []
    missing = [l for l in GITIGNORE_LINES if l not in existing]
    if missing:
        print("  .gitignore +", missing)
        if not dry:
            gi.write_text("\n".join(existing + missing) + "\n", encoding="utf-8")

    print("== 5. Nettoyage de l'ancien dossier")
    if not dry:
        if git_out("ls-files", "simu/base_holo-prog").strip():
            print("  des fichiers suivis subsistent dans l'ancien dossier : non supprimé.")
        else:
            shutil.rmtree("simu/base_holo-prog", ignore_errors=True)
            print("  simu/base_holo-prog supprimé (vide)")

    if not dry:
        run(["git", "add", "-A", str(NEW), ".gitignore"], False)
        print("\nTerminé. Vérifie avec `git status`, puis :")
        print('  git commit -m "Réorganisation de base_holo (core/legacy/examples/tests/tools/docs)"')
    else:
        print("\nDry-run terminé : rien n'a été modifié.")


if __name__ == "__main__":
    main()
