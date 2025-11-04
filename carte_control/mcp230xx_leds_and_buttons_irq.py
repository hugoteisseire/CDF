# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import board
import busio
from digitalio import DigitalInOut, Direction, Pull

from adafruit_mcp230xx.mcp23017 import MCP23017

# Initialize the I2C bus:
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the MCP23017 chip on the bonnet
mcp1 = MCP23017(i2c, address=0x20)
mcp2 = MCP23017(i2c, address=0x21)


# Make a list of all the port A pins (a.k.a 0-7)
port_a_pins = []
for pin in range(0, 8):
    port_a_pins.append(mcp1.get_pin(pin))
    port_a_pins.append(mcp2.get_pin(pin))
    

# Make a list of all the port B pins (a.k.a 8-15)
port_b_pins = []
for pin in range(8, 16):
    port_b_pins.append(mcp1.get_pin(pin))
    port_b_pins.append(mcp2.get_pin(pin))

# Set all the port A pins to output
for pin in port_a_pins:
    pin.direction = Direction.OUTPUT

# Set all the port B pins to input, with pullups!
for pin in port_b_pins:
    pin.direction = Direction.INPUT
    pin.pull = Pull.UP

# Set up to check all the port B pins (pins 8-15) w/interrupts!
mcp1.interrupt_enable = 0xFF00  # INTerrupt ENable top 8 bits
#dans mon cas pour le 1 :110011000000000

#dans mon cas pour le 2 :000000010000000
# If intcon is set to 0's we will get interrupts on
# both button presses and button releases
mcp1.interrupt_configuration = 0x0000  # interrupt on any change

# Or, we can ask to be notified CONTINUOUSLY if a pin goes LOW (button press)
# we won't get an IRQ pulse when the pin is HIGH!
# mcp.interrupt_configuration = 0xFF00         # notify pin value
# mcp.default_value = 0xFF00         # default value is 'high' so notify whenever 'low'

# connect the IRQ B pin to D4
irq_b = DigitalInOut(board.D4)

while True:
    if not irq_b.value:
        print("IRQ B went off")
        for num, button in enumerate(port_b_pins):
            if not button.value:
                print("Button #", num, "pressed!")
                # turn on matching port A pin
                port_a_pins[num].value = True  # turn LED on!
            else:
                port_a_pins[num].value = False  # turn LED off
