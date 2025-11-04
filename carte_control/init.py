# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import threading
import board
import busio
from digitalio import Direction, Pull
from adafruit_mcp230xx.mcp23017 import MCP23017

# ================================
# Initialisation I2C et MCP23017
# ================================

i2c = busio.I2C(board.SCL, board.SDA)
mcp1 = MCP23017(i2c, address=0x20)
mcp2 = MCP23017(i2c, address=0x21)

# ================================
# Fonctions de callback (stubs)
# ================================

def set_team_feed_back():
    global team
    if team:
        team_rgb_b.value = False
        team_rgb_r.value = True
    else:
        team_rgb_b.value = True
        team_rgb_r.value = False

def set_color_lidar(rouge, vert, bleu):
    lidar_rgb_r.value = rouge
    lidar_rgb_g.value = vert
    lidar_rgb_b.value = bleu

def set_color_state(rouge, vert, bleu):
    state_rgb_r.value = rouge
    state_rgb_g.value = vert
    state_rgb_b.value = bleu

def on_tirette_falling():
    print("Tirette retirée")

def on_bp_rst_lidar_rising():
    print("BP_RST_lidar appuyé")

def on_bp_init_rising():
    print("BP_init appuyé")

def on_bp_rst_state_rising():
    print("BP_RST_state appuyé")

def on_sw_team_change():
    global team
    team = SW_team.value
# ================================
# Définition des pins MCP1
# ================================
team = False
# --- MCP1 Port A : sorties RGB ---
team_rgb_b = mcp1.get_pin(0)
team_rgb_r = mcp1.get_pin(1)
lidar_rgb_g = mcp1.get_pin(2)
lidar_rgb_b = mcp1.get_pin(3)
lidar_rgb_r = mcp1.get_pin(4)
state_rgb_b = mcp1.get_pin(5)
state_rgb_g = mcp1.get_pin(6)
state_rgb_r = mcp1.get_pin(7)

output_pins = [
    team_rgb_b, team_rgb_r,
    lidar_rgb_g, lidar_rgb_b, lidar_rgb_r,
    state_rgb_b, state_rgb_g, state_rgb_r
]
for pin in output_pins:
    pin.direction = Direction.OUTPUT

# --- MCP1 Port B : entrées / interruptions ---
SW_spare_1 = mcp1.get_pin(8)
SW_spare_2 = mcp1.get_pin(9)
Tirette = mcp1.get_pin(10)
BP_RST_lidar = mcp1.get_pin(11)
BP_spare_1 = mcp1.get_pin(12)
BP_spare_2 = mcp1.get_pin(13)
BP_init = mcp1.get_pin(14)
BP_RST_state = mcp1.get_pin(15)

input_pins = [
    SW_spare_1, SW_spare_2, Tirette, BP_RST_lidar,
    BP_spare_1, BP_spare_2, BP_init, BP_RST_state
]
for pin in input_pins:
    pin.direction = Direction.INPUT
    pin.pull = Pull.DOWN


# ================================
# Définition des pins MCP2
# ================================

SW_team = mcp2.get_pin(0)
Buzzer = mcp2.get_pin(1)
SW_sel_1 = mcp2.get_pin(2)
SW_sel_2 = mcp2.get_pin(3)
SW_sel_3 = mcp2.get_pin(4)

# Sorties
Buzzer.direction = Direction.OUTPUT

# Entrées avec pull-down
for pin in [SW_team, SW_sel_1, SW_sel_2, SW_sel_3]:
    pin.direction = Direction.INPUT
    pin.pull = Pull.DOWN

# ================================
# Boucle principale (exemple)
# ================================

print("Configuration des MCP23017 terminée !")


def read_callback():
    SW_team_last = SW_team.value
    BP_init_last = BP_init.value
    BP_RST_state_last = BP_RST_state.value
    Tirette_last = Tirette.value
    BP_RST_lidar_last = BP_RST_lidar.value

    while True:
        if SW_team.value and SW_team_last != SW_team.value:
            set_team_feed_back()
        if BP_init.value and BP_init_last != BP_init.value:
            on_bp_init_rising()
        if BP_RST_state.value and BP_RST_state_last != BP_RST_state.value:
            on_bp_rst_state_rising()
        if Tirette.value and Tirette_last != Tirette.value:
            on_tirette_falling()
        if BP_RST_lidar.value and BP_RST_lidar_last != BP_RST_lidar.value:
            on_bp_rst_lidar_rising()
        SW_team_last = SW_team.value
        BP_init_last = BP_init.value    
        BP_RST_state_last = BP_RST_state.value
        Tirette_last = Tirette.value
        BP_RST_lidar_last = BP_RST_lidar.value  




try:
    stop_event = threading.Event()
    read_callback_task = threading.Thread(target=read_callback, args=(stop_event,))
    read_callback_task.start()
    
    while True:
        time.sleep(1)
    
except KeyboardInterrupt:
    print("🛑 Interruption par l'utilisateur")
finally:
    stop_event.set()  # Signale aux threads de s'arrêter
    read_callback_task.join()  # Attend la fin du thread
    print("🔴 Programme terminé")

