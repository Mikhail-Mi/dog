import network
import espnow
import json
import time
from machine import ADC, Pin

# -----------------------
# SETUP: Wi-Fi & ESP-NOW Init
# -----------------------
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

e = espnow.ESPNow()
e.init()

# Need to replace MAC 
peer_mac = b'\x24\x6f\x28\xaa\xbb\xcc'
e.add_peer(peer_mac)

# -----------------------
# SETUP: Joystick Pins
# -----------------------

# Movement Joystick (Left)
joy_left_x = ADC(Pin(34))  # VRx
joy_left_y = ADC(Pin(35))  # VRy

# Turning Joystick (Right)
joy_right_x = ADC(Pin(32))  # VRx only

# Configure ADC for 3.3V range
for joystick in [joy_left_x, joy_left_y, joy_right_x]:
    joystick.atten(ADC.ATTN_11DB)

# -----------------------
# HELPER: Normalize ADC
# -----------------------
def normalize(value, center=2048):
    return round((value - center) / 2048, 2)  # Returns -1.0 to +1.0

# -----------------------
# LOOP: Send Joystick Data
# -----------------------
while True:
    # Read raw ADC values
    lx = joy_left_x.read()
    ly = joy_left_y.read()
    rx = joy_right_x.read()

    # Normalize to range [-1.0, 1.0]
    move_x = normalize(lx)
    move_y = normalize(ly)
    turn = normalize(rx)

    # Package data
    data = {
        "move_x": move_x,
        "move_y": move_y,
        "turn": turn
    }

    # Convert to JSON and send
    e.send(peer_mac, json.dumps(data))

    print("Sent:", data)

    # Limit send rate
    time.sleep(0.05)  # 20 times/sec
