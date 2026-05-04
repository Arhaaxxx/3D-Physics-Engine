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

def object_collisions(b1, b2, restitution=0.8):
    delta = b2.position - b1.position
    dist = delta.magnitude()
    min_dist = b2.radius + b1.radius

    if dist < min_dist and dist > 0:
        normal = delta * (1.0 / dist)

        penetration = min_dist - dist
        b1.position = b1.position - normal * (penetration/2)
        b2.position = b2.position + normal * (penetration/2)

        rel_v = b2.velocity - b1.velocity
        vAlongNormal = rel_v.x * normal.x + rel_v.y * normal.y + rel_v.z * normal.z

        if vAlongNormal > 0:
            return
        
        j = -(1 + restitution) * vAlongNormal
        j /= (1 / b1.mass + 1 / b2.mass)

        impulse = normal * j

        b1.velocity = b1.velocity - impulse * (1 / b1.mass)
        b2.velocity = b2.velocity + impulse * (1 / b2.mass)

        contact_point = normal * b1.radius

        torque = Vector3(
            contact_point.y * impulse.z - contact_point.z * impulse.y,
            contact_point.z * impulse.x - contact_point.x * impulse.z,
            contact_point.x * impulse.y - contact_point.y * impulse.x,
        )

        b1.angular_velocity = b1.angular_velocity + torque * 0.1

def capsule_sphere_collision(capsule, sphere):
    p1 = capsule.p1.position
    p2 = capsule.p2.position
    c  = sphere.position

    # segment direction
    d = p2 - p1
    length_sq = d.dot(d)

    if length_sq == 0:
        return None

    # project sphere center onto segment
    t = (c - p1).dot(d) / length_sq
    t = max(0.0, min(1.0, t))

    closest = p1 + d * t

    delta = c - closest
    dist = delta.magnitude()

    if dist == 0:
        return None

    if dist < (sphere.radius + capsule.p1.radius):
        return closest, delta, dist

    return None

def capsule_sphere_response(capsule, sphere):

    result = capsule_sphere_collision(capsule, sphere)
    if result is None:
        return

    closest, delta, dist = result

    normal = delta * (1.0 / dist)

    # -------------------------
    # PENETRATION RESOLUTION
    # -------------------------
    penetration = (sphere.radius + capsule.p1.radius) - dist

    if penetration <= 0:
        return

    # mass-based correction
    total_mass = sphere.mass + capsule.p1.mass + capsule.p2.mass

    sphere_move = normal * (penetration * (capsule.p1.mass + capsule.p2.mass) / total_mass)
    cap_move   = normal * (penetration * sphere.mass / total_mass)

    sphere.position += sphere_move
    capsule.p1.position -= cap_move * 0.5
    capsule.p2.position -= cap_move * 0.5

    # -------------------------
    # VELOCITY (IMPULSE)
    # -------------------------
    cap_vel = (capsule.p1.velocity + capsule.p2.velocity) * 0.5

    rel_vel = sphere.velocity - cap_vel
    vel_along_normal = rel_vel.dot(normal)

    if vel_along_normal > 0:
        return

    restitution = 0.4

    j = -(1 + restitution) * vel_along_normal
    j /= (1/sphere.mass + 1/(capsule.p1.mass + capsule.p2.mass))

    impulse = normal * j

    # apply impulse
    sphere.velocity += impulse * (1 / sphere.mass)

    capsule.p1.velocity -= impulse * (0.5 / capsule.p1.mass)
    capsule.p2.velocity -= impulse * (0.5 / capsule.p2.mass)

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