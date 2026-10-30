#!/usr/bin/env bash
# Lance le serveur web et les deux simulateurs dans trois terminaux distincts.
set -euo pipefail

VIZ_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$VIZ_DIR"

PYTHON="${PYTHON:-python3}"
if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "Erreur : python3 est introuvable."
    exit 1
fi

# Socket locale partagee entre le simulateur robot et la strategie.
export CDF_ROBOT_SOCKET="${TMPDIR:-/tmp}/cdf_robot.sock"

# Utilise le terminal graphique de Raspberry Pi OS si disponible.
if command -v lxterminal >/dev/null 2>&1; then
    launch() { lxterminal --title="$1" --command="bash -lc 'cd "$VIZ_DIR"; export CDF_ROBOT_SOCKET="$CDF_ROBOT_SOCKET"; $2; echo; read -rp "Termine. Entree pour fermer..."'"; }
elif command -v x-terminal-emulator >/dev/null 2>&1; then
    launch() { x-terminal-emulator -T "$1" -e bash -lc "cd '$VIZ_DIR'; export CDF_ROBOT_SOCKET='$CDF_ROBOT_SOCKET'; $2; echo; read -rp 'Termine. Entree pour fermer...'"; }
else
    echo "Aucun terminal graphique detecte."
    if command -v tmux >/dev/null 2>&1; then
        SESSION="cdf-simulation"
        tmux new-session -d -s "$SESSION" -c "$VIZ_DIR" "$PYTHON server.py"
        tmux new-window -t "$SESSION" -n robot -c "$VIZ_DIR" "$PYTHON example_robot.py"
        tmux new-window -t "$SESSION" -n strategie -c "$VIZ_DIR" "$PYTHON example_strategie.py"
        echo "Simulation lancee dans tmux. Pour l'afficher : tmux attach -t $SESSION"
        echo "Pour l'arreter : tmux kill-session -t $SESSION"
    else
        echo "Installe tmux (sudo apt install tmux), puis relance ce script."
        exit 1
    fi
    exit 0
fi

launch "CDF - Serveur web" "$PYTHON server.py"
sleep 2
launch "CDF - Robot simule" "$PYTHON example_robot.py"
sleep 1
launch "CDF - Strategie simulee" "$PYTHON example_strategie.py"

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
echo "Les trois programmes sont lances."
echo "Depuis un appareil du meme reseau, ouvrir :"
echo "  http://${IP:-ADRESSE_IP_DE_LA_RASPBERRY}:8000"
echo
echo "Pour connaitre l'adresse IP : hostname -I"
