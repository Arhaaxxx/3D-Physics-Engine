from engine.vector import *
from engine.bodies.body import *
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
