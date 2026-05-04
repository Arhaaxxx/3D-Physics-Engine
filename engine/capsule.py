from engine.vector import *
from engine.body import *

class Capsule:
    def __init__(self, position, height, mass, radius=0.4):
        half = height / 2
        self.radius = radius

        self.p1 = Body(
            mass=mass/2,
            position=position + Vector3(0, 0, half),
            velocity=Vector3(),
            radius=radius
        )

        self.p2 = Body(
            mass=mass/2,
            position=position - Vector3(0, 0, half),
            velocity=Vector3(),
            radius=radius
        )

        self.rest_length = height


def solve_capsule_constraint(capsule):
    delta = capsule.p2.position - capsule.p1.position
    dist = delta.magnitude()

    if dist == 0:
        return

    diff = (dist - capsule.rest_length) / dist
    correction = delta * 0.5 * diff

    capsule.p1.position += correction
    capsule.p2.position -= correction