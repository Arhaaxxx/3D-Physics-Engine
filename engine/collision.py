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

def object_collisions(b1, b2):

    delta = b2.position - b1.position
    dist = delta.magnitude()

    if dist == 0:
        return

    normal = delta * (1.0 / dist)
    min_dist = b1.radius + b2.radius

    penetration = min_dist - dist

    if penetration > 0:

        # ---------- POSITION CORRECTION (BAUMGARTE) ----------
        percent = 0.2     # small correction
        slop = 0.01       # ignore tiny overlaps

        correction_mag = max(penetration - slop, 0.0) * percent
        correction = normal * correction_mag

        b1.position -= correction * 0.5
        b2.position += correction * 0.5

        # ---------- RELATIVE VELOCITY ----------
        rel_v = b2.velocity - b1.velocity
        vn = rel_v.dot(normal)

        # Do not resolve if separating
        if vn > 0:
            return

        # ---------- NORMAL IMPULSE ----------
        restitution = 0.2   # realistic bounce

        inv_mass1 = 1.0 / b1.mass
        inv_mass2 = 1.0 / b2.mass

        j = -(1 + restitution) * vn
        j /= (inv_mass1 + inv_mass2)

        impulse = normal * j

        b1.velocity -= impulse * inv_mass1
        b2.velocity += impulse * inv_mass2

        # ---------- FRICTION IMPULSE ----------
        rel_v = b2.velocity - b1.velocity  # recompute after normal impulse

        tangent = rel_v - normal * rel_v.dot(normal)

        if tangent.magnitude() > 1e-6:
            tangent = tangent.normalize()

            jt = -rel_v.dot(tangent)
            jt /= (inv_mass1 + inv_mass2)

            mu = 0.4   # friction coefficient

            # Coulomb friction clamp
            jt = max(-j * mu, min(jt, j * mu))

            friction_impulse = tangent * jt

            b1.velocity -= friction_impulse * inv_mass1
            b2.velocity += friction_impulse * inv_mass2

        # ---------- ANGULAR EFFECT (BASIC) ----------
        # Optional but improves realism
        r1 = normal * b1.radius
        r2 = normal * b2.radius

        torque1 = Vector3(
            r1.y * impulse.z - r1.z * impulse.y,
            r1.z * impulse.x - r1.x * impulse.z,
            r1.x * impulse.y - r1.y * impulse.x
        )

        torque2 = Vector3(
            r2.y * impulse.z - r2.z * impulse.y,
            r2.z * impulse.x - r2.x * impulse.z,
            r2.x * impulse.y - r2.y * impulse.x
        )

        b1.angular_velocity -= torque1 * inv_mass1 * 0.1
        b2.angular_velocity += torque2 * inv_mass2 * 0.1

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

def sphere_capsule_collision(body, cap):

    p1, p2 = get_capsule_endpoints(cap)

    # closest point on segment
    ab = p2 - p1
    t = (body.position - p1).dot(ab) / ab.dot(ab) if ab.dot(ab) > 1e-6 else 0
    t = max(0.0, min(1.0, t))
    closest = p1 + ab * t

    delta = body.position - closest
    dist = delta.magnitude()

    min_dist = body.radius + cap.radius

    if dist < min_dist and dist > 1e-6:

        normal = delta / dist
        penetration = min_dist - dist

        # ---------- POSITION CORRECTION ----------
        total_inv_mass = body.inv_mass + cap.inv_mass
        correction = normal * (penetration / total_inv_mass)

        body.position += correction * body.inv_mass
        cap.position -= correction * cap.inv_mass

        # ---------- RELATIVE VELOCITY ----------
        rel_v = body.velocity - cap.velocity
        vn = rel_v.dot(normal)

        if vn > 0:
            return

        restitution = 0.2

        j = -(1 + restitution) * vn
        j /= total_inv_mass

        impulse = normal * j

        # ---------- APPLY ----------
        body.velocity += impulse * body.inv_mass
        cap.velocity -= impulse * cap.inv_mass

        # ---------- ANGULAR EFFECT ----------
        r = closest - cap.position
        torque = Vector3(
            r.y * impulse.z - r.z * impulse.y,
            r.z * impulse.x - r.x * impulse.z,
            r.x * impulse.y - r.y * impulse.x
        )

        cap.angular_velocity -= torque * cap.inv_inertia