import math
import numpy as np

class Robot:
    def __init__(self, pos, angle, radius, wheel_angles):
        self.pos = np.array(pos, dtype=float)
        self.angle = angle
        self.radius = radius
        self.wheel_angles = wheel_angles
        self.direction_history = []  # à initialiser une fois, par exemple à l'init


def compute_wheel_speeds_global(robot, vx, vy, omega):
    cos_a, sin_a = math.cos(-robot.angle), math.sin(-robot.angle)
    v_local = np.array([
        cos_a * vx - sin_a * vy,
        sin_a * vx + cos_a * vy
    ])
    speeds = []
    for theta in robot.wheel_angles:
        speed = -math.sin(theta) * v_local[0] + math.cos(theta) * v_local[1] + omega * robot.radius
        speeds.append(speed)
    max_speed = max(abs(s) for s in speeds)
    if max_speed > 1:
        speeds = [s / max_speed for s in speeds]
    return speeds

def compute_base_velocity(robot, wheel_speeds):
    vx, vy = 0, 0
    for i, theta in enumerate(robot.wheel_angles):
        vx += -math.sin(theta) * wheel_speeds[i]
        vy += math.cos(theta) * wheel_speeds[i]
    return np.array([vx, vy])

def compute_rotation_velocity(robot, wheel_speeds):
    return sum(wheel_speeds) / (3 * robot.radius)

def rotate_vector(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

def shortest_angle_diff(target, current):
    diff = (target - current + math.pi) % (2 * math.pi) - math.pi
    return diff
