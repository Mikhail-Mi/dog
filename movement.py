import time
import math
import json
import network
from machine import I2C, Pin
from pca9685 import PCA9685
from servo import Servo
from ble_simple_peripheral import BLEPeripheral  # You'll need a custom BLE wrapper or library

# --- Setup ---
print("MAC:", ':'.join(['%02x' % b for b in network.WLAN(network.STA_IF).config('mac')]))

# I2C for PCA9685
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
pwm = PCA9685(i2c)
pwm.freq(50)

# Setup servos (Example: 12 total for 4 legs with 3 joints each)
servos = [Servo(pwm, ch) for ch in range(12)]

# BLE Setup
ble = BLEPeripheral(name="RobotDog")
ble.start()

# Gait and movement
gait_sequence = ["FR", "BL", "FL", "BR"]
current_leg_index = 0
current_state = "standing"
currently_walking = False
last_step_time = time.ticks_ms()

# Default foot positions (simple placeholder)
def default_position(leg):
    return {"hip": 90, "knee": 90, "tilt": 90}

foot_positions = {leg: default_position(leg) for leg in gait_sequence}

# --- Logic Blocks ---

# Define Lengths
L1 = 60  # length of thigh (mm)
L2 = 80  # length of shin (mm)

# Inverse Kinematics 
def inverse_kinematics(x, z):
    dist = math.sqrt(x**2 + z**2)
    if dist > (L1 + L2):
        dist = L1 + L2

    cos_knee = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
    knee_angle = math.acos(cos_knee)

    cos_alpha = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
    alpha = math.acos(cos_alpha)

    hip_angle = math.atan2(z, x) + alpha

    hip_deg = math.degrees(hip_angle)
    knee_deg = math.degrees(knee_angle)

    return hip_deg, knee_deg

def step_leg(servo_hip, servo_knee, start_pos, end_pos, steps=10, delay=0.05):
    for i in range(steps + 1):
        t = i / steps
        x = start_pos[0] + (end_pos[0] - start_pos[0]) * t
        z = start_pos[1] + (end_pos[1] - start_pos[1]) * t

        hip_angle, knee_angle = inverse_kinematics(x, z)
        servo_hip.angle(hip_angle)
        servo_knee.angle(knee_angle)
        time.sleep(delay)

def shift_weight(stance_legs):
    for leg in stance_legs:
        leg_index = gait_sequence.index(leg)
        tilt_servo = servos[leg_index * 3 + 0]
        tilt_servo.angle(80 if "L" in leg else 100)

def reset_weight():
    for i in range(0, 12, 3):
        tilt_servo = servos[i]
        tilt_servo.angle(90)

def turnInPlace(turn_value):
    pass

def angledMovement(move_x, move_y, turn_value):
    pass

def moveStraight(move_x, move_y):
    global current_leg_index, gait_sequence, servos

    leg_name = gait_sequence[current_leg_index]
    print("Stepping leg:", leg_name)

    leg_index = gait_sequence.index(leg_name)
    base = leg_index * 3
    tilt_servo = servos[base + 0]
    hip_servo = servos[base + 1]
    knee_servo = servos[base + 2]

    stance_legs = [l for l in gait_sequence if l != leg_name]
    shift_weight(stance_legs)

    magnitude = math.sqrt(move_x**2 + move_y**2)
    if magnitude > 1e-2:
        move_x /= magnitude
        move_y /= magnitude
    else:
        move_x = 0
        move_y = 0

    step_distance = 30
    lift_height = 40

    dx = move_x * step_distance
    dz = move_y * step_distance

    stand_z = -50
    start_pos = (0, stand_z)
    lift_pos = (0, stand_z - lift_height)
    move_pos = (dx, stand_z - lift_height)
    end_pos = (dx, stand_z)

    step_leg(hip_servo, knee_servo, start_pos, lift_pos)
    step_leg(hip_servo, knee_servo, lift_pos, move_pos)
    step_leg(hip_servo, knee_servo, move_pos, end_pos)

    reset_weight()
    current_leg_index = (current_leg_index + 1) % len(gait_sequence)

# --- Main Loop ---
while True:
    if ble.is_connected():
        data = ble.receive()
        if data:
            try:
                command = json.loads(data)
                move_x = command.get("move_x", 0)
                move_y = command.get("move_y", 0)
                turn = command.get("turn", 0)

                if abs(move_x) < 0.1 and abs(move_y) < 0.1 and abs(turn) > 0.1:
                    turnInPlace(turn)
                elif abs(move_x) > 0.1 or abs(move_y) > 0.1:
                    if abs(turn) > 0.1:
                        angledMovement(move_x, move_y, turn)
                    else:
                        moveStraight(move_x, move_y)

            except Exception as e:
                print("Parse error:", e)

    time.sleep_ms(20)
