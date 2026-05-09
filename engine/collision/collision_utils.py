from engine.capsule import *
from engine.vector import *
from engine.funcs import *

def ground_collision(body, restitution=0.3):

    if body.position.z - body.radius < 0:

        body.position.z = body.radius

        if body.velocity.z < 0:
            body.velocity.z *= -restitution

        # friction
        body.velocity.x *= 0.9
        body.velocity.y *= 0.9


def plane_collision(body, normal, offset, restitution=0.3):
    # distance from plane
    dist = body.position.dot(normal) - offset

    if dist < body.radius:
        # push out
        penetration = body.radius - dist
        body.position += normal * penetration

        # velocity response
        vel_normal = body.velocity.dot(normal)

        if vel_normal < 0:
            body.velocity -= normal * (1 + restitution) * vel_normal

def box_collision(body):

    # floor
    plane_collision(body, Vector3(0,0,1), 1)

    # ceiling
    plane_collision(body, Vector3(0,0,-1), -11)

    # left
    plane_collision(body, Vector3(1.5,0,0), -10)

    # right
    plane_collision(body, Vector3(-1.5,0,0), -10)

    # front
    plane_collision(body, Vector3(0,1.5,0), -10)

    # back
    plane_collision(body, Vector3(0,-1.5,0), -10)


def get_capsule_endpoints(cap):
    axis = cap.orientation.normalize()
    p1 = cap.position + axis * cap.half_length
    p2 = cap.position - axis * cap.half_length
    return p1, p2

def closest_point_on_segment(a, b, p):
    ab = b - a
    t = (p - a).dot(ab) / ab.dot(ab)
    t = max(0.0, min(1.0, t))
    return a + ab * t