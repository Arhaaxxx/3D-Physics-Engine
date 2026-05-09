from engine.vector import *
from engine.body import *
import math


class RigidCapsule:
    def __init__(self, position, height, radius, mass):

        self.position = position  # center
        self.velocity = Vector3()

        # rotate local Y axis into world Z axis
        self.orientation = [
            0.7071,
            0.7071,
            0,
            0
        ] # this is for quaternion w, x, y, z
        
        self.angular_velocity = Vector3()

        self.radius = radius
        self.half_length = height * 0.5

        self.mass = mass
        self.inv_mass = 1.0 / mass

        # Approx inertia (capsule ~ cylinder)
        I = (1/12) * mass * (3*radius*radius + height*height)
        self.inertia = I
        self.inv_inertia = 1.0 / I

def get_capsule_endpoints(cap):
    axis = cap.orientation.normalize()
    p1 = cap.position + axis * cap.half_length
    p2 = cap.position - axis * cap.half_length
    return p1, p2

def solve_capsules_array(self):
    for i in range(len(self.cap_pos1)):

        p1 = self.cap_pos1[i]
        p2 = self.cap_pos2[i]

        delta = p2 - p1
        dist = np.linalg.norm(delta)

        if dist < 1e-6:
            continue

        direction = delta / dist
        error = dist - self.cap_rest_length[i]

        correction = 0.5 * error * direction

        self.cap_pos1[i] += correction
        self.cap_pos2[i] -= correction

def closest_point_on_segment(a, b, p):
    ab = b - a
    t = (p - a).dot(ab) / ab.dot(ab)
    t = max(0.0, min(1.0, t))
    return a + ab * t